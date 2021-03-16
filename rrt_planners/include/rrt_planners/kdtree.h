#ifndef __KDTREE_H__
#define __KDTREE_H__


#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <numeric>
#include <functional>
#include <map>


// namespace kdt
//{
template <typename T>
class KDTree
{
public:
  /** @brief constructors
  */
  KDTree() : root_(nullptr), dims_(2), size_(0){};
  KDTree(size_t d) : root_(nullptr), dims_(d), size_(0){};
  KDTree(const KDTree&) = delete;
  KDTree& operator=(const KDTree&) = delete;
  KDTree(const std::vector<T>& points, size_t d) : root_(nullptr), dims_(d)
  {
    buildTree(points);
  }

  /** @brief destructor
  */
  ~KDTree()
  {
    delete root_;
  }

  /** @brief clear the kdtree structure
  */
  void clear()
  {
    delete root_;
    size_ = 0;
  }

  int size()
  {
    return size_;
  }

  /** @brief fill the vector with the elements of the structure.
  */
  void list(std::vector<T>& data) const
  {
    list(root_, data);
  }

  /** @brief add a new element
  */
  void add(const T& point)
  {
    root_ = addRecursive(root_, point, 0);
  };

  /** @brief Build the kdtree structure
  */
  void buildTree(const std::vector<T>& points)
  {
    buildRecursive(points);
  };

  /** @brief Searches the nearest neighbor.
  */
  T nearest(const T& point) const
  {
    if (root_ == nullptr)
      throw std::logic_error("No nearest! KDTree is empty!");
    // T* closer = nullptr;
    T closer;
    float min_dist = std::numeric_limits<double>::max();
    nearestNeighbor(root_, point, closer, min_dist, 0);
    return closer;
  }

  /** @brief Searches k-nearest neighbors.
  */
  std::vector<T> knearest(const T& point, int k) const
  {
    std::map<float, T> queue;
    knnSearchRecursive(root_, point, queue, k, 0);

    std::vector<T> indices;
    typename std::map<float, T>::iterator it = queue.begin();
    for (; it != queue.end(); it++)
    {
      indices.push_back(it->second);
    }
    return indices;
  }

  /** @brief Searches neighbors within radius.
  */
  std::vector<T> radiusSearch(const T& point, double radius) const
  {
    std::vector<T> indices;
    radiusSearchRecursive(root_, point, indices, radius, 0);
    return indices;
  }


private:
  struct KdTreeNode
  {
    KdTreeNode() : tpoint_(), left_(nullptr), right_(nullptr)
    {
    }
    KdTreeNode(const T& point) : tpoint_(point), left_(nullptr), right_(nullptr)
    {
    }
    ~KdTreeNode()
    {
      delete left_;
      delete right_;
    }

    double distance(const T& pt) const
    {
      return tpoint_.distance(pt);
    }

    T tpoint_;
    KdTreeNode* left_;
    KdTreeNode* right_;
  };

  // KD-Tree
  KdTreeNode* root_;
  size_t dims_;
  int size_;



  void list(KdTreeNode* n, std::vector<T>& data) const
  {
    if (n == nullptr)
      return;
    data.push_back(n->tpoint_);
    list(n->left_, data);
    list(n->right_, data);
  }

  void buildRecursive(const std::vector<T>& points)
  {
    for (unsigned int i = 0; i < points.size(); i++)
    {
      root_ = addRecursive(root_, points[i], 0);
    }
  }

  KdTreeNode* addRecursive(KdTreeNode* root, const T& point, int depth)
  {
    if (root == nullptr)
    {
      size_++;
      return new KdTreeNode(point);
    }
    // Calculate current dimension (cd) of comparison
    int cd = depth % dims_;

    if (point[cd] < root->tpoint_[cd])
    {
      root->left_ = addRecursive(root->left_, point, depth + 1);
    }
    else
    {
      root->right_ = addRecursive(root->right_, point, depth + 1);
    }
    return root;
  }

  void nearestNeighbor(KdTreeNode* root, const T& point, T& closest, float& mindist,
                       int depth = 0) const
  {
    if (root == nullptr)
      return;

    float current_distance = point.distance(root->tpoint_);

    if (current_distance < mindist)
    {
      mindist = current_distance;
      closest = root->tpoint_;
    }
    if (mindist == 0)
      return;

    int cd = depth % dims_;
    float plane_dist = (root->tpoint_[cd] - point[cd]);
    if (plane_dist > 0)
      nearestNeighbor(root->left_, point, closest, mindist, depth + 1);
    else
      nearestNeighbor(root->right_, point, closest, mindist, depth + 1);

    if ((plane_dist * plane_dist) >= mindist)
      return;

    if (plane_dist > 0)
      nearestNeighbor(root->right_, point, closest, mindist, depth + 1);
    else
      nearestNeighbor(root->left_, point, closest, mindist, depth + 1);
  }

  /** @brief Searches k-nearest neighbors recursively.
  */
  void knnSearchRecursive(KdTreeNode* root, const T& point, std::map<float, T>& queue,
                          int k, int depth) const
  {
    if (root == nullptr)
      return;

    float current_distance = point.distance(root->tpoint_);
    queue.insert(std::make_pair(current_distance, root->tpoint_));
    if ((int)queue.size() > k)
    {
      queue.erase(--queue.end());
    }

    int cd = depth % dims_;
    float plane_dist = (root->tpoint_[cd] - point[cd]);
    if (plane_dist > 0)
      knnSearchRecursive(root->left_, point, queue, k, depth + 1);
    else
      knnSearchRecursive(root->right_, point, queue, k, depth + 1);

    typename std::map<float, T>::iterator it = queue.end();
    if ((int)queue.size() < k || (plane_dist * plane_dist) < ((--it)->first))
    {
      if (plane_dist > 0)
        knnSearchRecursive(root->right_, point, queue, k, depth + 1);
      else
        knnSearchRecursive(root->left_, point, queue, k, depth + 1);
    }
  }


  /** @brief Searches all the neighbors in a radius recursively.
  */
  void radiusSearchRecursive(KdTreeNode* root, const T& point, std::vector<T>& vec,
                             float radius, int depth) const
  {
    if (root == nullptr)
      return;

    float distance = point.distance(root->tpoint_);
    if (distance < radius)
      vec.push_back(root->tpoint_);

    int cd = depth % dims_;
    float plane_dist = (root->tpoint_[cd] - point[cd]);
    if (plane_dist > 0)
      radiusSearchRecursive(root->left_, point, vec, radius, depth + 1);
    else
      radiusSearchRecursive(root->right_, point, vec, radius, depth + 1);

    if ((plane_dist * plane_dist) < radius)
    {
      if (plane_dist > 0)
        radiusSearchRecursive(root->right_, point, vec, radius, depth + 1);
      else
        radiusSearchRecursive(root->left_, point, vec, radius, depth + 1);
    }
  }
};


//}  // kdt

#endif  // !__KDTREE_H_
