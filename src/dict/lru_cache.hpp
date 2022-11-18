/*!
 * \file lru_cache.hpp
 *
 * \brief LRU cache implementation that provides synchronization between memory
 * and storage
 */
#pragma once
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cyy/naive_lib/log/log.hpp>
#include <cyy/naive_lib/util/runnable.hpp>
#include <spdlog/fmt/fmt.h>

#include "../thread_safe_container.hpp"
#include "ordered_dict.hpp"

namespace cyy::algorithm {
  template <typename Key, typename T> class storage_backend {
  public:
    using key_type = Key;
    using mapped_type = T;
    virtual ~storage_backend() = default;
    virtual std::vector<key_type> get_keys() = 0;
    virtual bool contains(const key_type &key) = 0;
    virtual std::optional<mapped_type> load_data(const key_type &key) = 0;
    virtual bool save_data(const key_type &key, mapped_type value) = 0;
    virtual void erase_data(const key_type &key) = 0;
    virtual void clear() = 0;
    virtual size_t batch_process_size() const { return 1; }
    virtual std::unordered_map<key_type, mapped_type>
    batch_load_data(const std::vector<key_type> &keys) {
      std::unordered_map<key_type, mapped_type> res;
      for (auto const &key : keys) {
        auto data_opt = load_data(key);
        if (data_opt.has_value()) {
          res.emplace(key, std::move(data_opt.value()));
        }
      }
      return res;
    }
    virtual std::vector<std::pair<key_type, bool>>
    batch_save_data(std::vector<std::pair<key_type, mapped_type>> batch_data) {
      std::vector<std::pair<key_type, bool>> batch_res;
      for (auto &[k, v] : batch_data) {
        auto res = save_data(k, std::move(v));
        batch_res.emplace_back(std::move(k), res);
      }
      return batch_res;
    }
  };
  template <typename Key, typename T> class lru_cache {
  public:
    using key_type = Key;
    using mapped_type = T;

    class value_reference final {
    public:
      value_reference(key_type key_, mapped_type value_,
                      lru_cache<key_type, mapped_type> *lru_cache_ptr_)
          : key(key_), value{value_}, lru_cache_ptr{lru_cache_ptr_} {}

      value_reference(const value_reference &) = delete;
      value_reference &operator=(const value_reference &) = delete;

      value_reference(value_reference &&rhs) noexcept {
        *this = std::move(rhs);
      }
      value_reference &operator=(value_reference &&rhs) noexcept {
        key = std::move(rhs.key);
        value = std::move(rhs.value);
        lru_cache_ptr = rhs.lru_cache_ptr;
        rhs.lru_cache_ptr = nullptr;
        return *this;
      }

      ~value_reference() {
        if (lru_cache_ptr) {
          lru_cache_ptr->emplace(key, value);
        }
      }
      const key_type &get_key() const { return key; }
      void cancel_writeback() { lru_cache_ptr = nullptr; }
      mapped_type &operator->() { return value; }

    private:
      key_type key;
      mapped_type value;
      lru_cache<key_type, mapped_type> *lru_cache_ptr{};
    };

  public:
    lru_cache(std::unique_ptr<storage_backend<key_type, mapped_type>> backend_)
        : backend(std::move(backend_)) {
      auto cpu_num = std::jthread::hardware_concurrency();
      set_saving_thread_number(cpu_num);
      set_fetch_thread_number(cpu_num);
      LOG_WARN("saving_thread_num and fetch_thread_num {}", cpu_num);
    }

    lru_cache(const lru_cache &) { LOG_WARN("stub function"); }
    lru_cache &operator=(const lru_cache &) = default;

    lru_cache(lru_cache &&) noexcept = delete;
    lru_cache &operator=(lru_cache &&) noexcept = delete;

    virtual ~lru_cache() { release(); }

    void release() {
      if (permanent) {
        flush(true);
      }
      fetch_request_queue.clear();
      save_request_queue.clear();
      data_dict.clear();
      data_info.clear();
      saving_data.clear();

      if (!permanent) {
        backend->clear();
      }
    }

    void emplace(const key_type &key, mapped_type value) {
      std::unique_lock lk(data_mutex);
      data_dict.emplace(key, std::move(value));
      data_info[key] = data_state::MEMORY_MODIFIED;
      dirty_data.emplace(key);
      hold_data.erase(key);
      saving_data.erase(key);
      if (data_dict.size() > in_memory_number) {
        auto wait_threshold =
            static_cast<size_t>(in_memory_number * wait_flush_ratio);
        lk.unlock();
        flush_expired_data();
        auto remain_size = save_request_queue.size();
        if (remain_size > wait_threshold) {
          LOG_DEBUG("wait flush remain_size is {} wait threshold is {} ",
                    remain_size, wait_threshold);
          save_request_queue.wait_for_less_size(in_memory_number,
                                                std::chrono::seconds(1));
        }
      }
    }

    std::optional<mapped_type> get(const key_type &key, bool hold = false) {
      std::unique_lock lk(data_mutex);
      while (true) {
        auto [result, value_opt] = prefetch(key, false);
        if (result < 0) {
          throw std::runtime_error(fmt::format("failed to get {}", key));
        }
        if (result > 0) {
          if (hold && value_opt.has_value()) {
            hold_data.emplace(key);
          }
          return value_opt;
        }
        LOG_DEBUG("wait data {}, fetch_request_queue size is {}", key,
                  fetch_request_queue.size());
        new_data_cv.wait(lk);
      }
      throw std::runtime_error("should not be here");
    }

    std::optional<value_reference> mutable_get(const key_type &key) {
      auto value_opt = get(key, true);
      if (value_opt.has_value()) {
        return value_reference(key, value_opt.value(), this);
      }
      return {};
    }

    size_t size() const {
      std::lock_guard lk(data_mutex);
      if (load_all_keys) {
        return data_info.size();
      }
      return keys().size();
    }
    void erase(const key_type &key) {
      std::lock_guard lk(data_mutex);
      if (!data_info.erase(key)) {
        return;
      }
      dirty_data.erase(key);
      hold_data.erase(key);
      data_dict.erase(key);
      saving_data.erase(key);
      backend->erase_data(key);
    }

    bool contains(const key_type &key) const {
      std::lock_guard lk(data_mutex);
      auto res = data_info.contains(key);
      if (!res && !load_all_keys) {
        res = backend->contains(key);
      }
      return res;
    }

    std::vector<key_type> keys() const {
      std::vector<key_type> res;
      std::lock_guard lk(data_mutex);
      if (!load_all_keys) {
        res = backend->get_keys();
        for (auto const &key : res) {
          if (!data_info.contains(key)) {
            data_info[key] = data_state::CONSISTENT;
          }
        }
        load_all_keys = true;
        return res;
      }

      res.reserve(data_info.size());
      for (auto const &[key, __] : data_info) {
        res.emplace_back(key);
      }
      return res;
    }

    std::vector<key_type> in_memory_keys() const {
      std::vector<key_type> res;
      std::lock_guard lk(data_mutex);
      res.reserve(data_dict.size());
      for (auto const &[key, _] : data_dict) {
        res.emplace_back(key);
      }
      return res;
    }
    void flush_expired_data(bool wait = false) {
      auto tasks = pop_expired_data(SIZE_MAX);
      flush_tasks(tasks);

      if (wait) {
        save_request_queue.wait_for_less_size(0, std::chrono::minutes(1));
      }
      return;
    }

    void flush(bool wait = false) {
      std::list<save_task> dirty_tasks;
      std::unique_lock lk(data_mutex);
      for (auto const &key : dirty_data) {
        auto it = data_info.find(key);
        if (it == data_info.end()) {
          continue;
        }
        if (hold_data.contains(key)) {
          continue;
        }
        if (it->second == data_state::MEMORY_MODIFIED) {
          it->second = data_state::PRE_SAVING;
          dirty_tasks.emplace_back(save_task{key});
        }
      }
      flush_tasks(dirty_tasks);
      if (wait) {
        save_request_queue.wait_for_less_size(0, std::chrono::years(1));
      }
    }

    void clear() {
      std::lock_guard lk(data_mutex);
      data_info.clear();
      data_dict.clear();
      saving_data.clear();
      backend->clear_data();
    }
    void prefetch(const std::vector<key_type> &keys) {
      for (auto const &key : keys) {
        prefetch(key);
      }
    }
    void set_in_memory_number(size_t in_memory_number_) {
      LOG_WARN("set in_memory_number {}", in_memory_number_);
      std::lock_guard lk(data_mutex);
      in_memory_number = in_memory_number_;
    }
    size_t get_in_memory_number() const {
      std::lock_guard lk(data_mutex);
      return in_memory_number;
    }
    void set_wait_flush_ratio(float wait_flush_ratio_) {
      std::lock_guard lk(data_mutex);
      wait_flush_ratio = wait_flush_ratio_;
    }
    void set_saving_thread_number(size_t saving_thread_num_) {
      if (saving_thread_num_ == 0) {
        throw std::runtime_error("saving_thread_num_ is 0");
      }

      std::lock_guard lk(data_mutex);
      if (saving_thread_num_ < saving_thread_num) {
        save_request_queue.clear();
        saving_thread_num = 0;
      }
      for (size_t i = saving_thread_num; i < saving_thread_num_; i++) {
        saving_threads.emplace_back(*this, i);
      }
      saving_thread_num = saving_thread_num_;
      LOG_DEBUG("new saving_thread_num {}", saving_thread_num);
    }
    void set_fetch_thread_number(size_t fetch_thread_num_) {
      if (fetch_thread_num_ == 0) {
        throw std::runtime_error("fetch_thread_num_ is 0");
      }
      std::lock_guard lk(data_mutex);
      if (fetch_thread_num_ < fetch_thread_num) {
        fetch_request_queue.clear();
        fetch_thread_num = 0;
      }
      for (size_t i = fetch_thread_num; i < fetch_thread_num_; i++) {
        fetch_threads.emplace_back(*this, i);
      }
      fetch_thread_num = fetch_thread_num_;
      LOG_DEBUG("new fetch_thread_num {}", fetch_thread_num);
    }

    void enable_permanent_storage() { permanent = true; }
    void disable_permanent_storage() { permanent = false; }

  protected:
    std::unique_ptr<storage_backend<key_type, mapped_type>> backend;

  private:
    mutable std::recursive_mutex data_mutex;
    enum class data_state : int {
      MEMORY_MODIFIED = 0,
      CONSISTENT,
      PRE_SAVING,
      SAVING,
      PRE_LOAD,
      LOADING,
      LOAD_FAILED,
    };
    class fetch_thread final : public cyy::naive_lib::runnable {
    public:
      fetch_thread(lru_cache &dict_, size_t id_) : dict(dict_), id(id_) {
        start(fmt::format("fetch_thread {}", id));
      }
      ~fetch_thread() override { stop(); }

    private:
      void run(const std::stop_token &st) override {
        while (!needs_stop()) {
          auto key_opt =
              dict.fetch_request_queue.pop_front(std::chrono::minutes(1), st);
          if (!key_opt.has_value()) {
            continue;
          }
          auto const &key = key_opt.value();
          std::unique_lock lk(dict.data_mutex);
          if (!dict.change_state(key, data_state::PRE_LOAD,
                                 data_state::LOADING)) {
            dict.new_data_cv.notify_all();
            continue;
          }
          lk.unlock();
          std::optional<lru_cache::mapped_type> data_opt;
          try {
            data_opt = dict.backend->load_data(key);
          } catch (const std::exception &e) {
            LOG_ERROR("load {} raised exception:{}", key, e.what());
          }
          lk.lock();
          if (!data_opt.has_value()) {
            LOG_ERROR("load {} failed", key);
            dict.change_state(key, data_state::LOADING,
                              data_state::LOAD_FAILED);
          } else if (dict.change_state(key, data_state::LOADING,
                                       data_state::CONSISTENT)) {
            dict.data_dict.emplace(key, std::move(data_opt.value()));
          }
          dict.new_data_cv.notify_all();
        }
      }

    private:
      lru_cache &dict;
      size_t id;
    };

    class save_thread final : public cyy::naive_lib::runnable {
    public:
      save_thread(lru_cache &dict_, size_t id_) : dict(dict_), id(id_) {
        start(fmt::format("saving_thread {}", id));
      }
      ~save_thread() override { stop(); }

    private:
      void run(const std::stop_token &st) override {
        auto batch_size = dict.backend->batch_process_size();
        while (!needs_stop()) {
          auto tasks = dict.save_request_queue.batch_pop_front(
              batch_size, std::chrono::minutes(1), st);
          if (tasks.empty()) {
            continue;
          }
          std::vector<std::pair<key_type, mapped_type>> batch_data;
          batch_data.reserve(tasks.size());
          std::vector<std::pair<key_type, bool>> res;
          res.reserve(tasks.size());
          std::unique_lock lk(dict.data_mutex);
          for (auto key : tasks) {
            if (!dict.change_state(key, data_state::PRE_SAVING,
                                   data_state::SAVING)) {
              continue;
            }
            res.emplace_back(key, false);
            if (auto it = dict.saving_data.find(key);
                it != dict.saving_data.end()) {
              batch_data.emplace_back(std::move(key), std::move(it->second));
            } else if (auto it = dict.data_dict.find(key);
                       it != dict.data_dict.end()) {
              batch_data.emplace_back(std::move(key), it->second);
            } else {
              throw std::runtime_error(fmt::format(
                  "can't find data of {} to save, invariant error", key));
            }
          }
          if (batch_data.empty()) {
            continue;
          }
          lk.unlock();
          try {
            res = dict.backend->batch_save_data(std::move(batch_data));
          } catch (const std::exception &e) {
            LOG_ERROR("batch save failed:{}", e.what());
          }
          lk.lock();
          for (auto const &[key, key_res] : res) {
            if (key_res && dict.change_state(key, data_state::SAVING,
                                             data_state::CONSISTENT)) {
              dict.dirty_data.erase(key);
              dict.saving_data.erase(key);
            } else if (key_res &&
                       dict.change_state(key, data_state::SAVING,
                                         data_state::MEMORY_MODIFIED)) {
              dict.saving_data.erase(key);
            } else if (!dict.contains(key)) {
              dict.backend->erase_data(key);
            }
          }
        }
      }

    private:
      lru_cache &dict;
      size_t id;
    };

    std::unordered_map<key_type, data_state> data_info;

    bool change_state(const key_type &key, data_state old_state,
                      data_state new_state) {
      auto it = data_info.find(key);
      if (it == data_info.end()) {
        return false;
      }
      if (it->second != old_state) {
        LOG_DEBUG("change_state failed old_state {}, state {} new state {}",
                  int(old_state), int(it->second), int(new_state));
        return false;
      }
      it->second = new_state;
      return true;
    }

    std::pair<int, std::optional<mapped_type>> prefetch(const key_type &key,
                                                        bool with_lock = true) {
      {
        std::unique_lock lk(data_mutex, std::defer_lock);
        if (with_lock) {
          lk.lock();
        }
        auto it = data_info.find(key);
        if (it == data_info.end()) {
          if (!load_all_keys) {
            if (backend->contains(key)) {
              data_info[key] = data_state::CONSISTENT;
              it = data_info.find(key);
            } else {
              return {1, {}};
            }
          }
        }
        if (it->second == data_state::PRE_SAVING ||
            it->second == data_state::SAVING) {
          return {1, saving_data[key]};
        }
        if (auto data_it = data_dict.find(key); data_it != data_dict.end()) {
          return {1, data_it->second};
        }
        if (it->second == data_state::LOAD_FAILED) {
          return {-1, {}};
        }

        if (it->second == data_state::LOADING) {
          return {0, {}};
        }
        if (it->second == data_state::PRE_LOAD) {
          return {0, {}};
        }
        if (it->second != data_state::CONSISTENT) {
          LOG_ERROR("invalid data_state {} for fetch {}", int(it->second), key);
          return {-1, {}};
        }
        it->second = data_state::PRE_LOAD;
      }
      // jump to queue front
      fetch_request_queue.emplace_front(fetch_task{key});
      return {0, {}};
    }
    using save_task = key_type;
    std::list<lru_cache::save_task> pop_expired_data(size_t max_number) {
      std::list<save_task> expired_data;
      while (expired_data.size() < max_number) {
        key_type key;
        T value;
        {
          std::unique_lock lk(data_mutex);
          if (data_dict.empty() || (max_number != SIZE_MAX &&
                                    data_dict.size() <= in_memory_number)) {
            break;
          }
          std::tie(key, value) = data_dict.pop_oldest();
          auto it = data_info.find(key);
          if (it == data_info.end()) {
            throw std::runtime_error(std::string("can't find info :" + key));
          }
          if (it->second == data_state::CONSISTENT) {
            continue;
          }
          if (hold_data.contains(key)) {
            data_dict.emplace(std::move(key), std::move(value));
            continue;
          }
          if (it->second != data_state::MEMORY_MODIFIED) {
            throw std::runtime_error(
                fmt::format("invalid state {} of key:{}",
                            std::to_string(static_cast<int>(it->second)), key));
          }
          it->second = data_state::PRE_SAVING;
          saving_data[key] = value;
        }
        expired_data.emplace_back(save_task{key});
      }
      return expired_data;
    }

    void flush_tasks(std::list<save_task> &tasks) {
      for (auto &task : tasks) {
        save_request_queue.emplace_back(std::move(task));
      }
    }

    cyy::algorithm::ordered_dict<key_type, mapped_type> data_dict;
    std::unordered_map<key_type, mapped_type> saving_data;
    std::unordered_set<key_type> hold_data;
    std::unordered_set<key_type> dirty_data;
    size_t in_memory_number{128};
    bool permanent{true};

    using save_request_queue_type =
        cyy::algorithm::thread_safe_linear_container<std::list<save_task>>;
    save_request_queue_type save_request_queue;
    size_t saving_thread_num{0};
    std::list<save_thread> saving_threads;

    using fetch_task = key_type;
    using fetch_request_queue_type =
        cyy::algorithm::thread_safe_linear_container<std::list<fetch_task>>;
    fetch_request_queue_type fetch_request_queue;
    size_t fetch_thread_num{0};
    std::list<fetch_thread> fetch_threads;

    std::condition_variable_any new_data_cv;
    bool mutable load_all_keys{false};
    float wait_flush_ratio{1.5};
  };
} // namespace cyy::algorithm
