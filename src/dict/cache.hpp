/*!
 * \file cache.hpp
 *
 * \brief A dictionary that provides synchronization between memory and storage
 */
#pragma once
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cyy/naive_lib/log/log.hpp>
#include <cyy/naive_lib/util/runnable.hpp>
#include <spdlog/fmt/fmt.h>

#include "../thread_safe_container.hpp"
#include "lru_cache.hpp"

namespace cyy::algorithm {
  template <typename Key, typename T> class storage_backend {
  public:
    using key_type = Key;
    using mapped_type = T;
    virtual ~storage_backend() = default;
    virtual std::vector<key_type> get_keys() = 0;
    virtual bool contains(const key_type &key) = 0;
    virtual mapped_type load_data(const key_type &key) = 0;
    virtual void clear_data() = 0;
    virtual void erase_data(const key_type &key) = 0;
    virtual void save_data(const key_type &key, mapped_type value) = 0;
  };
  template <typename Key, typename T> class cache {
  public:
    using key_type = Key;
    using mapped_type = T;

    class value_reference final {
    public:
      value_reference(key_type key_, mapped_type value_,
                      cache<key_type, mapped_type> *cache_ptr_)
          : key(key_), value{value_}, cache_ptr{cache_ptr_} {}

      value_reference(const value_reference &) = delete;
      value_reference &operator=(const value_reference &) = delete;

      value_reference(value_reference &&rhs) noexcept {
        *this = std::move(rhs);
      }
      value_reference &operator=(value_reference &&rhs) noexcept {
        key = std::move(rhs.key);
        value = std::move(rhs.value);
        cache_ptr = rhs.cache_ptr;
        rhs.cache_ptr = nullptr;
        return *this;
      }

      ~value_reference() {
        if (cache_ptr) {
          cache_ptr->emplace(key, value);
        }
      }
      mapped_type &operator->() { return value; }

    private:
      key_type key;
      mapped_type value;
      cache<key_type, mapped_type> *cache_ptr{};
    };

  public:
    cache(std::unique_ptr<storage_backend<key_type, mapped_type>> backend_)
        : backend(std::move(backend_)) {
      cyy::naive_lib::log::set_level(spdlog::level::level_enum::warn);

      auto cpu_num = std::jthread::hardware_concurrency();
      set_saving_thread_number(cpu_num);
      set_fetch_thread_number(cpu_num);
      LOG_WARN("saving_thread_num and fetch_thread_num {}", cpu_num);
    }

    cache(const cache &) { LOG_WARN("stub function"); }
    cache &operator=(const cache &) = default;

    cache(cache &&) noexcept = delete;
    cache &operator=(cache &&) noexcept = delete;

    virtual ~cache() { release(); }

    void release() {
      if (permanent) {
        flush(SIZE_MAX, true);
      }
      fetch_request_queue.clear();
      save_request_queue.clear();
      data_cache.clear();
      data_info.clear();
      saving_data.clear();

      if (!permanent) {
        backend->clear_data();
      }
    }

    void emplace(const key_type &key, mapped_type value) {
      std::unique_lock lk(data_mutex);
      data_cache.emplace(key, std::move(value));
      data_info[key] = data_state::IN_MEMORY;
      saving_data.erase(key);
      if (data_cache.size() > in_memory_number) {
        auto wait_threshold =
            static_cast<size_t>(in_memory_number * wait_flush_ratio);
        lk.unlock();
        flush();
        auto old_in_memory_number = in_memory_number;
        auto remain_size = save_request_queue.size();
        if (remain_size > wait_threshold) {
          LOG_DEBUG("wait flush remain_size is {} wait threshold is {} ",
                    remain_size, wait_threshold);
          save_request_queue.wait_for_less_size(old_in_memory_number,
                                                std::chrono::seconds(1));
        }
      }
    }

    std::optional<mapped_type> get(const key_type &key) {
      std::unique_lock lk(data_mutex);
      while (true) {
        auto [result, value_opt] = prefetch(key, false);
        if (result < 0) {
          throw std::runtime_error(fmt::format("failed to get {}", key));
        }
        if (result > 0) {
          return value_opt;
        }
        LOG_DEBUG("wait data {}, fetch_request_queue size is {}", key,
                  fetch_request_queue.size());
        new_data_cv.wait(lk);
      }
      throw std::runtime_error("should not be here");
    }

    std::optional<value_reference> mutable_get(const key_type &key) {
      auto value_opt = get(key);
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
      data_cache.erase(key);
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
            data_info[key] = data_state::IN_DISK;
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
      res.reserve(data_cache.size());
      for (auto const &[key, _] : data_cache) {
        res.emplace_back(key);
      }
      return res;
    }
    void flush(size_t flush_num = SIZE_MAX, bool wait = false) {
      auto tasks = pop_expired_data(flush_num);
      flush(tasks);

      if (wait) {
        save_request_queue.wait_for_less_size(0, std::chrono::minutes(1));
      }
      return;
    }
    void clear() {
      std::lock_guard lk(data_mutex);
      data_info.clear();
      data_cache.clear();
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
      IN_MEMORY = 0,
      IN_DISK,
      PRE_SAVING,
      SAVING,
      PRE_LOAD,
      LOADING,
      LOAD_FAILED,
    };
    class fetch_thread final : public cyy::naive_lib::runnable {
    public:
      fetch_thread(cache &dict_, size_t id_) : dict(dict_), id(id_) {
        start(fmt::format("fetch_thread {}", id));
      }
      ~fetch_thread() override { stop(); }

    private:
      void run(const std::stop_token &st) override {
        while (!needs_stop()) {
          auto value_opt =
              dict.fetch_request_queue.pop_front(std::chrono::minutes(1), st);
          if (!value_opt.has_value()) {
            continue;
          }
          auto const &key = value_opt.value();
          try {
            {
              std::lock_guard lk(dict.data_mutex);
              if (!dict.change_state(key, data_state::PRE_LOAD,
                                     data_state::LOADING)) {
                dict.new_data_cv.notify_all();
                continue;
              }
            }
            auto value = dict.backend->load_data(key);
            {
              std::lock_guard lk(dict.data_mutex);
              if (dict.change_state(key, data_state::LOADING,
                                    data_state::IN_DISK)) {
                dict.data_cache.emplace(key, std::move(value));
              }
            }
            dict.new_data_cv.notify_all();
          } catch (const std::exception &e) {
            LOG_ERROR("load {} failed:{}", key, e.what());
            {
              std::lock_guard lk(dict.data_mutex);
              if (!dict.change_state(key, data_state::LOADING,
                                     data_state::LOAD_FAILED)) {
                continue;
              }
            }
          }
        }
      }

    private:
      cache &dict;
      size_t id;
    };

    class save_thread final : public cyy::naive_lib::runnable {
    public:
      save_thread(cache &dict_, size_t id_) : dict(dict_), id(id_) {
        start(fmt::format("saving_thread {}", id));
      }
      ~save_thread() override { stop(); }

    private:
      void run(const std::stop_token &st) override {
        std::optional<cache::save_task> value_opt;
        while (!needs_stop()) {
          value_opt =
              dict.save_request_queue.pop_front(std::chrono::minutes(1), st);
          if (!value_opt.has_value()) {
            continue;
          }
          auto &key = value_opt.value();
          try {
            std::unique_lock lk(dict.data_mutex);
            if (!dict.change_state(key, data_state::PRE_SAVING,
                                   data_state::SAVING)) {
              continue;
            }
            auto value = dict.saving_data.at(key);
            lk.unlock();
            dict.backend->save_data(key, value);
            lk.lock();
            if (dict.change_state(key, data_state::SAVING,
                                  data_state::IN_DISK)) {
              dict.saving_data.erase(key);
            } else if (!dict.contains(key)) {
              dict.backend->erase_data(key);
            }
          } catch (const std::exception &e) {
            LOG_ERROR("save {} failed,drop it:{}", key, e.what());
            dict.erase(key);
          }
        }
      }

    private:
      cache &dict;
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
              data_info[key] = data_state::IN_DISK;
              it = data_info.find(key);
            } else {
              return {1, {}};
            }
          }
        }
        if (it->second == data_state::PRE_SAVING ||
            it->second == data_state::SAVING) {
          auto node = saving_data.extract(key);
          data_cache.emplace(key, node.mapped());
          it->second = data_state::IN_MEMORY;
          return {1, std::move(node.mapped())};
        }
        if (auto data_it = data_cache.find(key); data_it != data_cache.end()) {
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
        if (it->second != data_state::IN_DISK) {
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
    std::list<cache::save_task> pop_expired_data(size_t max_number) {
      std::list<save_task> expired_data;
      while (expired_data.size() < max_number) {
        key_type key;
        T value;
        {
          std::unique_lock lk(data_mutex);
          if (data_cache.empty() || (max_number != SIZE_MAX &&
                                     data_cache.size() <= in_memory_number)) {
            break;
          }
          std::tie(key, value) = data_cache.pop_oldest();
          auto it = data_info.find(key);
          if (it == data_info.end()) {
            throw std::runtime_error(std::string("can't find info :" + key));
          }
          if (it->second == data_state::IN_DISK) {
            continue;
          }
          if (it->second != data_state::IN_MEMORY) {
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
    void flush(std::list<save_task> &tasks) {
      for (auto &task : tasks) {
        save_request_queue.emplace_back(std::move(task));
      }
    }

    cyy::algorithm::lru_cache<key_type, mapped_type> data_cache;
    std::unordered_map<key_type, mapped_type> saving_data;
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
