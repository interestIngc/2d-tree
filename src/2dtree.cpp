#include "primitives.h"
#include <vector>
#include <unordered_set>
#include <functional>
#include <queue>
using namespace kdtree;

std::size_t PointSet :: size() const {
    return size_;
}
bool PointSet ::empty() const {
    return size_ == 0;
}
void PointSet::report(Node * a,  std::vector<Point> & ans) const {
    if (a == nullptr) {
        return;
    }
    report(a->left.get(), ans);
    ans.push_back(a->point);
    report(a->right.get(), ans);
}

void PointSet ::get_range(Node * a, const Rect &rect, std::vector<Point> & ans) const {
    if (rect.contains(a->point)) {
        ans.push_back(a->point);
    }
    if (a->left == nullptr and a->right == nullptr) {
        return;
    }
    if (a->left != nullptr) {
        Rect l = a->left->rect;
        if (rect.includes(l)) {
            report(a->left.get(), ans);
        } else if (rect.intersects(l)) {
            get_range(a->left.get(), rect, ans);
        }
    }
    if (a->right != nullptr) {
        Rect r = a->right->rect;
        if (rect.includes(r)) {
            report(a->right.get(), ans);
        } else if (rect.intersects(r)) {
            get_range(a->right.get(), rect, ans);
        }
    }
}
std::pair<PointSet::ForwardIt, PointSet::ForwardIt> PointSet :: range(const Rect & rect) const {
    std::vector<Point> ans;
    get_range(root_.get(), rect, ans);
    auto it = ans.begin();
    auto t = build_tree(it, it + ans.size(), 0, nullptr);
    auto ps = PointSet(std::move(t));
    return std::make_pair(ps.begin(), ps.end());
}

PointSet::Node* PointSet::find_node(Node* curr_node, const Point & pt) const {
    if (curr_node == nullptr || curr_node->point == pt) return curr_node;

    const Point & curr_point = curr_node->point;
    int currLevel = curr_node->level;
    if (pt.coords[currLevel % 2] < curr_point.coords[currLevel % 2]) {
        if (curr_node->left == nullptr) {
            return curr_node;
        } else {
            return find_node(curr_node->left.get(), pt);
        }
    } else {
        if (curr_node->right == nullptr) {
            return curr_node;
        } else {
            return find_node(curr_node->right.get(), pt);
        }
    }
}

bool PointSet::contains(const Point & pt) const {
    auto node = find_node(root_.get(), pt);
    return node != nullptr && node->point == pt;
}

void PointSet::put(const Point & pt) {
    auto new_node = find_node(root_.get(), pt);
    if (new_node == nullptr) {
        root_ = std::make_unique<Node>(pt, 0, Rect(pt, pt), nullptr);
        size_ = 1;
    } else {
        if (new_node->point == pt) {
            return;
        } else {
            int level = new_node->level;
            if (pt.coords[level % 2] < new_node->point.coords[level % 2]) {
                new_node->left = std::make_unique<Node>(pt, level + 1, Rect(pt, pt), new_node);
                recalc(new_node->left.get());
            } else {
                new_node->right = std::make_unique<Node>(pt, level + 1, Rect(pt, pt), new_node);
                recalc(new_node->right.get());
            }
            size_ ++;
        }
    }
}
Rect PointSet::update(PointSet::Node * currNode) const {
    double xmin0 = currNode->point.x();
    double xmax0 = currNode->point.x();
    double ymin0 = currNode->point.y();
    double ymax0 = currNode->point.y();
    if (currNode->left != nullptr) {
        xmin0 = fmin(currNode->left->rect.xmin(), xmin0);
        xmax0 = fmax(currNode->left->rect.xmax(), xmax0);
        ymin0 = fmin(currNode->left->rect.ymin(), ymin0);
        ymax0 = fmax(currNode->left->rect.ymax(), ymax0);
    }
    if (currNode->right != nullptr) {
        xmin0 = fmin(currNode->right->rect.xmin(), xmin0);
        xmax0 = fmax(currNode->right->rect.xmax(), xmax0);
        ymin0 = fmin(currNode->right->rect.ymin(), ymin0);
        ymax0 = fmax(currNode->right->rect.ymax(), ymax0);
    }
    Rect rect = Rect(Point(xmin0, ymin0), Point(xmax0, ymax0));
    return rect;
}

void PointSet::recalc(PointSet::Node * curr_node) {
    if (curr_node->parent == nullptr) {
        return;
    }
    curr_node = curr_node->parent;
    curr_node->rect = update(curr_node);
    return recalc(curr_node);
}
void PointSet::get_nearest_neighbour(const PointSet::Node* current_node, const Point & key,
                                     std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, cmp> & rev_queue, size_t k) const {
    if (current_node == nullptr) return;
    const Point & curr_point = current_node->point;
    rev_queue.push(std::pair<double, Point> (curr_point.distance(key), current_node->point));
    if (rev_queue.size() > k) {
        rev_queue.pop();
    }
    size_t curr_level = current_node->level;
    bool is_left;
    if (key.coords[curr_level % 2] < curr_point.coords[curr_level % 2]) {
        get_nearest_neighbour(current_node->left.get(), key, rev_queue, k);
        is_left = true;
    } else {
        get_nearest_neighbour(current_node->right.get(), key, rev_queue, k);
        is_left = false;
    }

    if (rev_queue.size() < k || fabs(key.coords[curr_level % 2] - curr_point.coords[curr_level % 2]) < rev_queue.top().first) {
        if (is_left) get_nearest_neighbour(current_node->right.get(), key, rev_queue, k);
        else get_nearest_neighbour(current_node->left.get(), key, rev_queue, k);
    }
}

std::optional<Point> PointSet::nearest(const Point & key) const {
    auto q = nearest(key, 1);
    if (q.second == q.first) {
        return std::nullopt;
    }
    Point res = q.first.node->point;
    return res;
}
std::pair<PointSet::ForwardIt, PointSet::ForwardIt> PointSet::nearest(const Point & key,
                                                                      std::size_t k) const {
    cmp comp;
    std::vector<Point> result;
    auto q = std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, cmp> (comp, {});
    get_nearest_neighbour(root_.get(), key, q, k);
    while(!q.empty()) {
        result.push_back(q.top().second);
        q.pop();
    }
    auto it = result.begin();
    auto tree = build_tree(it, it + result.size(), 0, nullptr);
    auto ps = PointSet(std::move(tree));
    return std::make_pair(ps.begin(), ps.end());
}

std::unique_ptr<PointSet::Node> PointSet::build_tree(std::vector<Point>::iterator start,
                                                     std::vector<Point>::iterator end, int curr_level, Node * parent) const {
    if (start >= end) return nullptr;
    int axis = curr_level % 2;
    auto cmp = [axis](const Point& p1, const Point& p2) {
        return p1.coords[axis] < p2.coords[axis];
    };
    std::size_t len = end - start;
    auto mid = start + len / 2;
    std::nth_element(start, mid, end, cmp);
    while (mid > start && (mid - 1)->coords[axis] == mid->coords[axis]) {
        --mid;
    }
    Point pt = *mid;
    auto new_node = std::make_unique<Node>(pt, curr_level, Rect(pt, pt), parent);
    new_node->left = build_tree(start, mid, curr_level + 1, new_node.get());
    new_node->right = build_tree(mid + 1, end, curr_level + 1, new_node.get());
    new_node->rect = update(new_node.get());
    return new_node;
}