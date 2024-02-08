#ifndef DSTAR_H
#define DSTAR_H

#include <cmath>
#include <vector>
#include <list>
#include <algorithm>
#include <exception>

#include <iostream>

namespace libdstar
{

struct state_point
{
    long x;
    long y;
    enum state_tag
    {
        stNew,
        stClosed,
        stOpen,
        stRaise,
        stLower,
        stObstacle
    } tag;
    long weight; // c
    long weight_previous;
    long cost_previous; // p
    long cost_actual; // h
    long k() const; // k
    double potential;
    state_point* backpointer; // b
    state_point();
    std::pair<double, double> float_coords();
};

struct state_point_comparator
{
    bool operator()(state_point *a, state_point *b);
};

class state_map
{
private:
    std::vector< std::vector<state_point*> > map;
    long width;
    long height;
public:
    state_map(const long& _width, const long& _height);
    ~state_map();
    inline state_point* operator()(const long& _x, const long& _y);
    std::vector<state_point*> neighbors(const state_point* point);
    std::vector<state_point*> neighbors(const long& _x, const long& _y);
    long get_width() const;
    long get_height() const;
    void reset();
    state_point* point(const long& _x, const long& _y);
};

class dstar_exception: public std::exception
{
private:
    std::string message;
public:
    dstar_exception(const std::string& msg);
    virtual const char* what() const noexcept override;
};

class dstar
{
private:
    state_map* map;
    state_point* origin;
    state_point* destination;
    std::list<state_point*> open_list;
    std::list<state_point*> draft_path;
    std::list<state_point*> reduced_path;
    std::list<state_point*> optimized_path;
    void reset();
    state_point* min_state();
    long get_kmin();
    void insert_state(state_point* point);
    void delete_state(state_point* point);
    long iterate_state();
    long point_modified(const long& _x, const long& _y, long cost, state_point::state_tag obstacle_state = state_point::stClosed);
    std::list<state_point*> trace_path();
    int cutoff_distance;
    int r_field;
    double repulsion_gain;
public:
    dstar(state_map* _map);
    void init_targets(const long& origin_x, const long& origin_y,
                      const long& destination_x, const long& destination_y,
                      const bool& weighted = true);
    void set_cutoff_distance(int value);
    void set_repulsion_gain(double value);
    void set_r_field(int value);
    std::list<state_point*> generate_trajectory();
    std::list<state_point*> replan_trajectory(const long& origin_x, const long& origin_y, const long& cost);
};

}

#endif
