#pragma once

#include <iostream>
#include <cmath>
#include <map>
#include <algorithm>
#include <optional>
#include <memory>
#include <utility>
#include <vector>
#include <set>
#include <queue>
#include <type_traits>
class Point {
public:

    Point(double x, double y)
        : coords{x, y}
    {}
    double coords[2];
    double x() const {
        return coords[0];
    }
    double y() const {
        return coords[1];
    }
    double distance(const Point & point) const {
        return sqrt(pow((this->x() - point.x()), 2) + pow(this->y() - point.y(), 2));
    }

    bool operator< (const Point & point) const {
        if (this->x() < point.x()) {
            return true;
        } else if (point.x() < this->x()) {
            return false;
        }
        return this->y() < point.y();
    }
    bool operator> (const Point & point) const {
        if (this->x() > point.x()) {
            return true;
        } else if (point.x() > this->x()) {
            return false;
        }
        return this->y() > point.y();
    }
    bool operator<= (const Point & point) const {
        if (this->x() < point.x()) {
            return true;
        } else if (point.x() < this->x()) {
            return false;
        }
        return this->y() <= point.y();
    }
    bool operator>= (const Point & point) const {
        if (this->x() > point.x()) {
            return true;
        } else if (point.x() > this->x()) {
            return false;
        }
        return this->y() >= point.y();
    }
    bool operator== (const Point & point) const {
        return this->x() == point.x() and this->y() == point.y();
    }
    bool operator!= (const Point & point) const {
        return !(this->x() == point.x() and this->y() == point.y());
    }

    friend std::ostream &operator<<(std::ostream &out, const Point &point) {
        out << "point {x: " << point.x() << "; y: " << point.y() << "}";
        return out;
    }
};

class Rect {
public:

    Rect(const Point & left_bottom, const Point & right_top) {
        this->xmin0 = left_bottom.x();
        this->ymin0 = left_bottom.y();
        this->xmax0 = right_top.x();
        this->ymax0 = right_top.y();
    }
    double xmin0;
    double ymin0;
    double xmax0;
    double ymax0;

    double xmin() const {
        return xmin0;
    }
    double ymin() const {
        return ymin0;
    }
    double xmax() const {
        return xmax0;
    }
    double ymax() const {
        return ymax0;
    };

    double distance(const Point &p) const {
        if (contains(p)) {
            return 0;
        }
        if (p.x() >= xmax() and p.y() >= ymax()) {
            return p.distance(Point(xmax(), ymax()));
        }
        if (p.x() >= xmax() and p.y() <= ymin()) {
            return p.distance(Point(xmax(), ymin()));
        }
        if (p.x() <= xmin() and p.y() <= ymin()) {
            return p.distance(Point(xmin(), ymin()));
        }
        if (p.x() <= xmin() and p.y() >= xmax()) {
            return p.distance(Point(xmin(), ymax()));
        }
        if (p.y() >= ymax()) {
            return p.y() - ymax();
        }
        if (p.x() >= xmax()) {
            return p.x() - xmax();
        }
        if (p.y() <= ymin()) {
            return ymin() - p.y();
        }
        return xmin() - p.x();
    }
    bool contains(const Point & p) const {
        return p.x() >= this->xmin() && p.x() <= this->xmax() && p.y() >= this->ymin() && p.y() <= this->ymax();
    }

    bool includes(const Rect & rectangle) const {
        return rectangle.xmin() >= this->xmin() and rectangle.xmax() <= this->xmax() and
               rectangle.ymin() >= this->ymin() and rectangle.ymax() <= this->ymax();
    }

    bool intersects(const Rect &other) const {
        return other.contains(Point(xmax(), ymax())) || other.contains(Point(xmin(), ymin())) ||
               contains(Point(other.xmax(), other.ymax())) || contains(Point(other.xmin(), other.ymin()));
    }
};
class cmp {
public:
    bool operator()(std::pair<double, Point> p1, std::pair<double, Point> p2) {
        return p1.first < p2.first;
    }
};
class iterator;
namespace rbtree {

    class PointSet {
    private:
        typedef std::set<Point> set;
        set points;
        class Iterate {
            friend class PointSet;

            std::shared_ptr<set> saved_set;
            set::iterator it;

        public:
            typedef std::forward_iterator_tag iterator_category;
            typedef ptrdiff_t difference_type;
            typedef const Point value_type;
            typedef value_type *pointer;
            typedef value_type &reference;

        public:
            Iterate() = default;

            Iterate(Iterate const &other) = default;

        private:
            explicit Iterate(const set::iterator &it, std::shared_ptr<set> saved_set = nullptr) : saved_set(std::move(saved_set)), it(it) {}

        public:
            reference operator*() {
                return it.operator*();
            }

            pointer operator->() const {
                return it.operator->();
            }

            Iterate &operator++() {
                it++;
                return *this;
            }

            Iterate operator++(int) {
                auto ans = *this;
                ++*this;
                return ans;
            }

            friend bool operator==(const Iterate &it1, const Iterate &it2) {
                return it1.it == it2.it;
            }

            friend bool operator!=(const Iterate &it1, const Iterate &it2) {
                return it1.it != it2.it;
            }
        };

    public:

        using ForwardIt = Iterate;

        bool empty() const {
            return points.empty();
        }

        std::size_t size() const {
            return points.size();
        }

        void put(const Point &p) {
            points.insert(p);
        }

        bool contains(const Point &p) const {
            return points.count(p) != 0;
        }


        std::pair<ForwardIt, ForwardIt> range(const Rect &r) const {
            auto current = std::make_shared<set>();
            for (auto it = begin(); it != end(); it++) {
                if (r.contains(*it)) {
                    current->insert(*it);
                }
            }
            return std::pair(Iterate(current->begin(), current), Iterate(current->end(), current));
        }

        ForwardIt begin() const {
            return Iterate(points.cbegin());
        }

        ForwardIt end() const {
            return Iterate(points.cend());
        }

        std::optional<Point> nearest(const Point &p) const {
            auto res = nearest(p, 1);
            if ((res.first == res.second && res.first == end())) {
                return std::nullopt;
            }
            return *res.first;
        }

        std::pair<ForwardIt, ForwardIt> nearest(const Point &p, std::size_t k) const {
            k = std::min(k, size());
            auto subset = std::make_shared<set>();
            std::set<std::pair<double, Point>> current;
            for (auto it = points.begin(); it != points.end(); it++) {
                auto dist = p.distance(*it);
                current.insert(std::pair<double, Point> (dist, *it));
            }
            auto it = current.begin();
            for (size_t i = 0; i < k; i++) {
                subset->insert((*it).second);
                it++;
            }
            return std::pair(Iterate(subset->begin(), subset), Iterate(subset->end(), subset));
        }

        friend std::ostream &operator<<(std::ostream &out, const PointSet &pointSet) {
            for (auto it = pointSet.begin(); it != pointSet.end(); ++it) {
                out << *it;
            }
            return out;
        }
    };

}

namespace kdtree {

    class PointSet {
        friend Point;
    private:
        struct Node {
            Point point;
            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            Node * parent;
            size_t level;
            Rect rect;
            Node(const Point & _pt, size_t _level, const Rect & _rect, Node * parent):
                    point(_pt), left(nullptr), right(nullptr), parent(parent), level(_level), rect(_rect) { };

        };
        class NodeIterate {
            friend class PointSet;

        public:
            typedef std::forward_iterator_tag iterator_category;
            typedef ptrdiff_t difference_type;
            using value_type = const Point;
            using pointer = value_type *;
            using reference = value_type &;

        private:
            Node * node;
            std::shared_ptr<Node> sub;
        public:
            NodeIterate() = default;

            NodeIterate(NodeIterate const &other) : node(other.node), sub(other.sub) {}

        private:
            explicit NodeIterate(Node * node, std::shared_ptr<Node> sub_tree = nullptr) : node(node), sub(std::move(sub_tree)) {}

        public:
            reference operator*() {
                return node->point;
            }

            pointer operator->() const {
                return &(node->point);
            }

            NodeIterate &operator++() {
                if (!node || node->parent == nullptr) {
                    node = nullptr;
                } else {
                     if (node->parent->left.get() == node) {
                         node = node->parent;
                        if (node->right != nullptr) {
                            node = node->right.get();
                            while (node->left != nullptr or node->right != nullptr) {
                                if (node->left != nullptr) {
                                    node = node->left.get();
                                } else {
                                    node = node->right.get();
                                }
                            }
                        }
                    } else {
                         node = node->parent;
                    }
                }
                return *this;
            }

            NodeIterate operator++(int) {
                auto ans = *this;
                ++*this;
                return ans;
            }

            friend bool operator==(const NodeIterate &it1, const NodeIterate &it2) {
                return it1.node == it2.node;
            }

            friend bool operator!=(const NodeIterate &it1, const NodeIterate &it2) {
                return !(it1 == it2);
            }
        };
    public:
        explicit PointSet(std::shared_ptr<Node> node = nullptr, int size = 0) : root_(std::move(node)), size_(size) {}

        std::shared_ptr<Node> root_;

        using ForwardIt = NodeIterate;

        bool empty() const;
        std::size_t size() const;
        void put(const Point &);
        bool contains(const Point &) const;

        [[nodiscard]] std::pair<ForwardIt, ForwardIt> range(const Rect & rect) const;
        [[nodiscard]] ForwardIt begin() const {
            if (root_ != nullptr) {
                Node * cur = root_.get();
                while (cur->left != nullptr || cur->right != nullptr) {
                    if (cur->left != nullptr) {
                        cur = cur->left.get();
                    } else {
                        cur = cur->right.get();
                    }
                }
                return NodeIterate(cur, root_);
            } else {
                return end();
            }
        }
        [[nodiscard]] ForwardIt end() const {
            return NodeIterate(nullptr);
        }

        [[nodiscard]] std::optional<Point> nearest(const Point &) const;
        [[nodiscard]] std::pair<ForwardIt, ForwardIt> nearest(const Point &, std::size_t) const;

        friend std::ostream & operator << (std::ostream & out, const PointSet & ps) {
            for (auto it = ps.begin(); it != ps.end(); ++it) {
                out << *it << "\n";
            }
            return out;
        }
    private:
        std::size_t size_ = 0;
        Node * find_node(Node * curr_node, const Point & pt) const;
        void get_nearest_neighbour(const Node* current_node, const Point & key, std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, cmp> & rev_queue, size_t k) const;
        std::unique_ptr<Node> build_tree(std::vector<Point>::iterator start, std::vector<Point>::iterator end, int curr_level, Node *parent) const;

        void report(Node * a, std::vector<Point> &ans) const;

        void get_range(Node * a, const Rect &rect, std::vector<Point> &ans) const;

        void recalc(Node * curr_node);

        Rect update(Node *currNode) const;
    };

}


