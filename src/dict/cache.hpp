/*!
 * \file cache.hpp
 *
 * \brief
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

#include "../thread_safe_container.hpp"
#include "ordered_dict.hpp"

namespace cyy::algorithm {
  template <typename T> class storage_backend {
  public:
    virtual ~storage_backend() = default;
    virtual std::vector<std::string> load_keys() = 0;
    virtual T load_data(const std::string &key) = 0;
    virtual void clear_data() = 0;
    virtual void erase_data(const std::string &key) = 0;
    virtual void save_data(const std::string &key, T value) = 0;
  };
  template <typename T> class cache {
  public:
    cache(std::unique_ptr<storage_backend<T>> backend_)
        : backend(std::move(backend_)) {
      cyy::naive_lib::log::set_level(spdlog::level::level_enum::warn);

      for (const auto key : backend->load_keys()) {
        data_info[key] = data_state::IN_DISK;
        LOG_DEBUG("load key {}", key);
      }
      if (data_info.empty()) {
        LOG_WARN("no key to load");
      } else {
        LOG_WARN("load {} keys", data_info.size());
      }

      auto cpu_num = std::jthread::hardware_concurrency();
      saving_thread_num = cpu_num;
      fetch_thread_num = cpu_num;
      LOG_WARN("saving_thread_num and fetch_thread_num {}", cpu_num);

      for (size_t i = 0; i < saving_thread_num; i++) {
        saving_threads.emplace_back(*this, i);
      }
      for (auto &t : saving_threads) {
        t.start();
      }
      for (size_t i = 0; i < fetch_thread_num; i++) {
        fetch_threads.emplace_back(*this);
      }
      for (auto &t : fetch_threads) {
        t.start();
      }
    }

    cache(const cache &rhs) { LOG_WARN("stub function"); }
    cache &operator=(const cache &) = default;

    cache(cache &&) noexcept = delete;
    cache &operator=(cache &&) noexcept = delete;

    virtual ~cache() { release(); }

    void release() {
      if (permanent) {
        flush_all(true);
      }
      for (size_t i = 0; i < fetch_thread_num; i++) {
        fetch_request_queue.emplace_back();
      }
      for (size_t i = 0; i < saving_thread_num; i++) {
        save_request_queue.emplace_back();
      }
      for (auto &t : fetch_threads) {
        t.stop();
      }
      for (auto &t : saving_threads) {
        t.stop();
      }
      fetch_request_queue.clear();
      save_request_queue.clear();
      data.clear();
      data_info.clear();
      saving_data.clear();

      if (!permanent) {
        backend->clear_data();
      }
    }

    void emplace(const std::string &key, const T &value) {
      std::unique_lock lk(data_mutex);
      data.emplace(key, value);
      data_info[key] = data_state::IN_MEMORY_NEW_DATA;
      saving_data.erase(key);
      if (data.size() > in_memory_number) {
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

    std::optional<T> get(const std::string &key) {
      std::unique_lock lk(data_mutex);
      while (true) {
        auto [result, value_opt] = prefetch(key, false);
        if (result < 0) {
          throw std::runtime_error(key);
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

    size_t size() const {
      std::lock_guard lk(data_mutex);
      return data_info.size();
    }
    void erase(const std::string &key) {
      std::lock_guard lk(data_mutex);
      if (!data_info.erase(key)) {
        return;
      }
      data.erase(key);
      saving_data.erase(key);
      backend->erase_data(key);
    }

    bool contains(const std::string &key) const {
      std::lock_guard lk(data_mutex);
      return data_info.find(key) != data_info.end();
    }

    void set_logging(bool enable_debug) const {
      if (enable_debug) {
        cyy::naive_lib::log::set_level(spdlog::level::level_enum::debug);
      } else {
        cyy::naive_lib::log::set_level(spdlog::level::level_enum::warn);
      }
    }

    std::vector<std::string> keys() const {
      std::vector<std::string> res;
      std::lock_guard lk(data_mutex);
      res.reserve(data_info.size());
      for (auto const &[key, __] : data_info) {
        res.emplace_back(key);
      }
      return res;
    }

    std::vector<std::string> in_memory_keys() const {
      std::vector<std::string> res;
      std::lock_guard lk(data_mutex);
      res.reserve(data_info.size());
      for (auto const &[key, state] : data_info) {
        if (state == data_state::IN_MEMORY ||
            state == data_state::IN_MEMORY_NEW_DATA) {
          res.emplace_back(key);
        }
      }
      return res;
    }
    void flush_all(bool wait = false) {
      std::unique_lock lk(data_mutex);
      auto old_in_memory_number = in_memory_number;
      in_memory_number = 0;
      auto tasks = pop_expired_data(SIZE_MAX);
      in_memory_number = old_in_memory_number;
      lk.unlock();
      if (tasks.empty()) {
        return;
      }
      flush(tasks);
      if (!wait) {
        return;
      }

      save_request_queue.wait_for_less_size(0, std::chrono::minutes(1));
      lk.lock();
      if (saving_data.empty()) {
        return;
      }
      flush_finished_cv.wait(lk);
    }
    void flush(size_t flush_num = SIZE_MAX) {
      auto tasks = pop_expired_data(flush_num);
      flush(tasks);
      return;
    }
    void clear() {
      std::lock_guard lk(data_mutex);
      data_info.clear();
      data.clear();
      saving_data.clear();
      backend->clear_data();
    }
    void prefetch(const std::vector<std::string> &keys) {
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

      std::unique_lock lk(data_mutex);
      while (saving_thread_num > saving_thread_num_) {
        save_request_queue.emplace_back();
        saving_thread_num--;
      }
      for (size_t i = saving_thread_num; i < saving_thread_num_; i++) {
        saving_threads.emplace_back(*this, i);
        saving_threads.back().start();
      }
      saving_thread_num = saving_thread_num_;
      LOG_WARN("new saving_thread_num {}", saving_thread_num);
    }
    void set_fetch_thread_number(size_t fetch_thread_num_) {
      if (fetch_thread_num_ == 0) {
        throw std::runtime_error("fetch_thread_num_ is 0");
      }
      std::unique_lock lk(data_mutex);
      while (fetch_thread_num > fetch_thread_num_) {
        fetch_request_queue.emplace_back();
        fetch_thread_num--;
      }
      for (size_t i = 0; i < fetch_thread_num_ - fetch_thread_num; i++) {
        fetch_threads.emplace_back(*this);
        fetch_threads.back().start();
      }
      fetch_thread_num = fetch_thread_num_;
      LOG_WARN("new fetch_thread_num {}", fetch_thread_num);
    }

    void enable_permanent_storage() { permanent = true; }
    void disable_permanent_storage() { permanent = false; }

    class fetch_thread final : public cyy::naive_lib::runnable {
    public:
      fetch_thread(cache &dict_) : dict(dict_) {}

    private:
      void run() override {
        while (!needs_stop()) {
          auto value_opt =
              dict.fetch_request_queue.pop_front(std::chrono::minutes(1));
          if (!value_opt.has_value()) {
            continue;
          }
          if (!(*value_opt).has_value()) {
            return;
          }
          auto const &key = value_opt.value().value();
          try {
            {
              std::lock_guard lk(dict.data_mutex);
              if (!dict.change_state(key, data_state::PRE_LOAD,
                                     data_state::LOADING)) {
                continue;
              }
            }
            auto value = dict.backend->load_data(key);
            {
              std::lock_guard lk(dict.data_mutex);
              if (!dict.change_state(key, data_state::LOADING,
                                     data_state::IN_MEMORY)) {
                continue;
              }
              dict.data.emplace(key, std::move(value));
            }
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
          dict.new_data_cv.notify_all();
        }
      }

    private:
      cache &dict;
    };

    class save_thread final : public cyy::naive_lib::runnable {
    public:
      explicit save_thread(cache &dict_, size_t id_) : dict(dict_), id(id_) {}

    private:
      void run() override {
        LOG_DEBUG("run save_thread id {}", id);
        std::optional<std::optional<cache::save_task>> value_opt;
        while (!needs_stop()) {
          if (id == 0) {
            value_opt = dict.save_request_queue.pop_front(
                std::chrono::milliseconds(500));
          } else {
            value_opt =
                dict.save_request_queue.pop_front(std::chrono::minutes(1));
          }
          if (!value_opt.has_value()) {
            if (id == 0) {
              dict.flush();
              LOG_DEBUG("flush by save thread");
            }
            continue;
          }
          if (!(*value_opt).has_value()) {
            return;
          }
          auto &key = value_opt.value().value();
          try {
            std::unique_lock lk(dict.data_mutex);
            if (!dict.change_state(key, data_state::PRE_SAVING,
                                   data_state::SAVING)) {
              continue;
            }
            auto value = dict.saving_data[key];
            lk.unlock();
            dict.backend->save_data(key, value);
            lk.lock();
            if (dict.change_state(key, data_state::SAVING,
                                  data_state::IN_DISK)) {
              dict.saving_data.erase(key);
              if (dict.saving_data.empty()) {
                lk.unlock();
                dict.flush_finished_cv.notify_all();
              }
              continue;
            }
            if (!dict.data_info.count(key)) {
              dict.backend->erase_data(key);
            }
          } catch (const std::exception &e) {
            LOG_ERROR("torch::save {} failed,drop it:{}", key, e.what());
            dict.erase(key);
          }
        }
      }

    private:
      cache &dict;
      size_t id;
    };

  protected:
    std::unique_ptr<storage_backend<T>> backend;

  private:
    enum class data_state : int {
      IN_MEMORY = 0,
      IN_MEMORY_NEW_DATA,
      IN_DISK,
      PRE_SAVING,
      SAVING,
      PRE_LOAD,
      LOADING,
      LOAD_FAILED,
    };
    std::unordered_map<std::string, data_state> data_info;
    mutable std::recursive_mutex data_mutex;

    bool change_state(const std::string &key, data_state old_state,
                      data_state new_state) {
      auto it = data_info.find(key);
      if (it == data_info.end()) {
        return false;
      }
      if (it->second != old_state) {
        LOG_ERROR("change_state failed old_state {}, state {} new state {}",
                  int(old_state), int(it->second), int(new_state));
        return false;
      }
      it->second = new_state;
      return true;
    }

    std::pair<int, std::optional<T>> prefetch(const std::string &key,
                                              bool with_lock = true) {
      {
        std::unique_lock lk(data_mutex, std::defer_lock);
        if (with_lock) {
          lk.lock();
        }
        auto it = data_info.find(key);
        if (it == data_info.end()) {
          return {1, {}};
        }
        if (it->second == data_state::PRE_SAVING ||
            it->second == data_state::SAVING) {
          auto node = saving_data.extract(key);
          data.emplace(key, node.mapped());
          it->second = data_state::IN_MEMORY_NEW_DATA;
          return {1, std::move(node.mapped())};
        }
        if (it->second == data_state::IN_MEMORY ||
            it->second == data_state::IN_MEMORY_NEW_DATA) {
          return {1, *data.find(key)};
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
    using save_task = std::string;
    std::list<cache::save_task> pop_expired_data(size_t max_number) {
      std::list<save_task> expired_data;
      while (expired_data.size() < max_number) {
        std::string key;
        T value;
        {
          std::unique_lock lk(data_mutex);

          if (data.size() <= in_memory_number) {
            break;
          }
          std::tie(key, value) = data.pop_front();
          auto it = data_info.find(key);
          if (it == data_info.end()) {
            throw std::runtime_error(std::string("can't find info :" + key));
          }
          if (it->second == data_state::IN_MEMORY) {
            it->second = data_state::IN_DISK;
            continue;
          }
          if (it->second != data_state::IN_MEMORY_NEW_DATA) {
            throw std::runtime_error(
                std::string("invalid state " +
                            std::to_string(static_cast<int>(it->second)) +
                            " of key:" + key));
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

    cyy::algorithm::ordered_dict<std::string, T> data;
    std::unordered_map<std::string, T> saving_data;
    size_t in_memory_number{128};
    bool permanent{true};

    using save_request_queue_type =
        cyy::algorithm::thread_safe_linear_container<
            std::list<std::optional<save_task>>>;
    save_request_queue_type save_request_queue;
    size_t saving_thread_num{8};
    std::list<save_thread> saving_threads;

    using fetch_task = std::string;
    using fetch_request_queue_type =
        cyy::algorithm::thread_safe_linear_container<
            std::list<std::optional<fetch_task>>>;
    fetch_request_queue_type fetch_request_queue;
    size_t fetch_thread_num{8};
    std::list<fetch_thread> fetch_threads;

    std::condition_variable_any new_data_cv;
    std::condition_variable_any flush_finished_cv;
    float wait_flush_ratio{1.5};
  };
} // namespace cyy::algorithm
