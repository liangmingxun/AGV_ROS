#pragma once

#include <mutex> // 添加互斥锁所需的头文件
#include <shared_mutex>
#include <functional>

template <typename DataType>
class SharedDataManager {
public:
    SharedDataManager() : data_(), mutex_() {}  // 默认构造函数

    SharedDataManager(const SharedDataManager& other) 
        : data_(other.data_){  // 只复制数据，不复制 mutex
    }

    SharedDataManager& operator=(const SharedDataManager& other) {
        if (this != &other) {
            data_ = other.data_;
        }
        return *this;
    }
    
    template <typename F>
    auto multiaction_with_unique_lock(F&& action) {
        std::unique_lock<std::shared_mutex> lock(mutex_); // 独占锁
        return std::invoke(std::forward<F>(action), data_);
    }

    template <typename F>
    auto multiaction_with_shared_lock(F&& action) {
        std::shared_lock<std::shared_mutex> lock(mutex_); // 共享锁
        return std::invoke(std::forward<F>(action), data_);
    }

private:
    mutable std::shared_mutex mutex_; // 共享互斥锁
    DataType data_;  // 存储数据的容器
};