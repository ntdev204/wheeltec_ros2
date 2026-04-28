


#ifndef COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_
#define COSTMAP_QUEUE__MAP_BASED_QUEUE_HPP_

#include <algorithm>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace costmap_queue
{


template<class item_t>
class MapBasedQueue
{
public:
  

  explicit MapBasedQueue(bool reset_bins = true)
  : reset_bins_(reset_bins), item_count_(0)
  {
    reset();
  }

  

  virtual void reset()
  {
    if (reset_bins_ || item_count_ > 0) {
      item_bins_.clear();
      item_count_ = 0;
    }
    iter_ = last_insert_iter_ = item_bins_.end();
  }

  

  void enqueue(const double priority, item_t item)
  {



    if (last_insert_iter_ == item_bins_.end() || last_insert_iter_->first != priority) {
      last_insert_iter_ = item_bins_.find(priority);


      if (last_insert_iter_ == item_bins_.end()) {
        auto map_item = std::make_pair(priority, std::move(std::vector<item_t>()));



        std::pair<ItemMapIterator, bool> insert_result = item_bins_.insert(std::move(map_item));
        last_insert_iter_ = insert_result.first;
      }
    }


    last_insert_iter_->second.push_back(item);
    item_count_++;


    if (iter_ == item_bins_.end() || priority < iter_->first) {
      iter_ = last_insert_iter_;
    }
  }

  

  bool isEmpty()
  {
    return item_count_ == 0;
  }

  

  item_t & front()
  {
    if (iter_ == item_bins_.end()) {
      throw std::out_of_range("front() called on empty costmap_queue::MapBasedQueue!");
    }

    return iter_->second.back();
  }

  

  void pop()
  {
    if (iter_ != item_bins_.end() && !iter_->second.empty()) {
      iter_->second.pop_back();
      item_count_--;
    }

    auto not_empty = [](const typename ItemMap::value_type & key_val) {
        return !key_val.second.empty();
      };
    iter_ = std::find_if(iter_, item_bins_.end(), not_empty);
  }

protected:
  using ItemMap = std::map<double, std::vector<item_t>>;
  using ItemMapIterator = typename ItemMap::iterator;

  bool reset_bins_;

  ItemMap item_bins_;
  unsigned int item_count_;
  ItemMapIterator iter_;
  ItemMapIterator last_insert_iter_;
};
}

#endif
