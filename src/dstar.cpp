#include "dstar.h"

using namespace libdstar;

long state_point::k() const
{
    if(tag == stOpen)
        return std::min(cost_actual, cost_previous);
    else
        return -1;
}

state_point::state_point()
{
    tag = stNew;
    cost_previous = -1;
    cost_actual = -1;
    weight = 0;
    weight_previous = 0;
    backpointer = nullptr;
    potential = 0;
}

std::pair<double, double> state_point::float_coords()
{
    return std::pair<double, double>(static_cast<double>(x), static_cast<double>(y));
}

bool state_point_comparator::operator()(state_point *a, state_point *b)
{
    return(a->k() < b->k());
}

state_map::state_map(const long &_width, const long &_height)
{
    width = _width;
    height = _height;
    map = std::vector< std::vector<state_point*> >(width);
    map.reserve(width);
    for(long i = 0; i < width; i++)
    {
        map[i] = std::vector<state_point*>(height);
        map[i].reserve(height);
    }
    for(long i = 0; i < width; i++)
        for(long j = 0; j < height; j++)
        {
            map[i][j] = new state_point;
            map[i][j]->x = i;
            map[i][j]->y = j;
        }
}

state_map::~state_map()
{
    for(long i = 0; i < width; i++)
        for(long j = 0; j < height; j++)
        {
            delete map[i][j];
        }
}

std::vector<state_point *> state_map::neighbors(const state_point *point)
{
    std::vector<state_point*> result;
    for(int i = 1; i >= -1; i--)
        for(int j = -1; j <= 1; j++)
        {
            long x_index = point->x + i;
            long y_index = point->y + j;
            if((x_index >= 0) &&
                    (x_index < width) &&
                    (y_index >= 0) &&
                    (y_index < height) &&
                    !((i == 0) && (j == 0)))
                result.push_back(map[x_index][y_index]);
        }
    return result;
}

std::vector<state_point *> state_map::neighbors(const long &_x, const long &_y)
{
    if((_x < width) && (_y < height))
        return neighbors(map[_x][_y]);
    return std::vector<state_point *>();
}

long state_map::get_width() const
{
    return width;
}

long state_map::get_height() const
{
    return height;
}

void state_map::reset()
{
    for(long i = 0; i < width; i++)
        for(long j = 0; j < height; j++)
        {
            map[i][j]->backpointer = nullptr;
            if(map[i][j]->tag != state_point::stObstacle)
                map[i][j]->tag = state_point::stNew;
            map[i][j]->potential = 0;
            map[i][j]->cost_actual = -1;
            map[i][j]->cost_previous = -1;
            map[i][j]->weight = map[i][j]->weight_previous;
        }
}

state_point *state_map::point(const long &_x, const long &_y)
{
    if((_x < width) && (_y < height))
        return map[_x][_y];
    return nullptr;
}

state_point *state_map::operator()(const long &_x, const long &_y)
{
    return point(_x, _y);
}

void dstar::reset()
{
    if(!open_list.empty())
        open_list.clear();
    map->reset();
}

state_point *dstar::min_state()
{
    if(open_list.empty())
        return nullptr;
    return open_list.front();;
}

long dstar::get_kmin()
{
    if(open_list.empty())
        return -1;
    return open_list.front()->k();
}

void dstar::insert_state(state_point *point)
{
    auto point_iterator = std::find(open_list.begin(), open_list.end(), point);
    if(point_iterator == open_list.end())
        open_list.push_back(point);
    point->tag = state_point::stOpen;
    open_list.sort(state_point_comparator());
    //std::sort(open_list.begin(), open_list.end(), dstar::state_sort_comparator);
}

void dstar::delete_state(state_point *point)
{
    auto point_iterator = std::find(open_list.begin(), open_list.end(), point);
    if(point_iterator != open_list.end())
    {
        open_list.erase(point_iterator);
        point->tag = state_point::stClosed;
        open_list.sort(state_point_comparator());
        //std::sort(open_list.begin(), open_list.end(), dstar::state_sort_comparator);
    }
}

long dstar::iterate_state()
{
    state_point* X = min_state();
    if((X == nullptr) || (X == origin)) return -1;
    int k_old = get_kmin();
    std::cout << "\rProcessing: [" << X->x << ", " << X->y << "] " <<
                 X->cost_actual <<
                 " OPEN list length " << open_list.size();
    delete_state(X);
    std::vector<state_point*>neighbors = map->neighbors(X);
    //std::cout << "Extracted " << neighbors.size() << " neighbors" << std::endl;
    for(auto i = neighbors.begin(); i < neighbors.end(); i++)
    {
        state_point* Y = (*i);
        if(Y->tag == state_point::stObstacle)
            continue;
        if(Y->tag == state_point::stClosed)
        {
            if((Y->cost_actual < k_old) && (X->cost_actual > (Y->cost_actual + Y->weight)))
            {
                X->backpointer = Y;
                X->cost_actual = Y->cost_actual + Y->weight;
            }
        }
    }
    for(auto i = neighbors.begin(); i < neighbors.end(); i++)
    {
        state_point* Y = (*i);
        if(Y->tag == state_point::stObstacle)
            continue;
        if(Y->tag == state_point::stNew)
        {
            Y->backpointer = X;
            Y->cost_actual = Y->weight + X->cost_actual;
            Y->cost_previous = Y->cost_actual;
            insert_state(Y);
        }
        else
        {
            if((Y->backpointer == X) && (Y->cost_actual != (X->cost_actual + X->weight)))
            {
                if(Y->tag == state_point::stOpen)
                {
                    if(Y->cost_actual < Y->cost_previous)
                        Y->cost_previous = Y->cost_actual;
                    Y->cost_actual = X->cost_actual + X->weight;
                }
                else
                {
                    Y->cost_actual = X->cost_actual + X->weight;
                    Y->cost_previous = Y->cost_actual;
                }
                insert_state(Y);
            }
            else
            {
                if((Y->backpointer != X) && (Y->cost_actual > (X->cost_actual + X->weight)))
                {
                    if(X->cost_previous >= X->cost_actual)
                    {
                        Y->backpointer = X;
                        Y->cost_actual = X->cost_actual + X->weight;
                        if(Y->tag == state_point::stClosed)
                            Y->cost_previous = Y->cost_actual;
                        insert_state(Y);
                    }
                    else
                    {
                        X->cost_previous = X->cost_actual;
                        insert_state(X);
                    }
                }
                else
                {
                    if((Y->backpointer != X) && (X->cost_actual > (Y->cost_actual + Y->weight)))
                    {
                        if((Y->tag == state_point::stClosed) && (Y->cost_actual > k_old))
                        {
                            Y->cost_previous = Y->cost_actual;
                            insert_state(Y);
                        }
                    }
                }
            }
        }
    }
    return get_kmin();
}

long dstar::point_modified(const long &_x, const long &_y, long cost, state_point::state_tag obstacle_state)
{
    if((_x < map->get_width()) && (_y < map->get_height()) && (map->point(_x, _y)->backpointer != nullptr))
    {
        state_point* a = map->point(_x, _y);
        if(a->tag == state_point::stClosed)
        {
            a->tag = obstacle_state;
            a->cost_previous = a->cost_actual;
            a->cost_actual = cost;
            insert_state(a);
        }
        return get_kmin();
    }
    return -1;
}

std::list<state_point *> dstar::trace_path()
{
    draft_path.clear();
    reduced_path.clear();
    optimized_path.clear();
    std::cout << "Constructing draft path..."<< std::endl;
    state_point* s = origin;
    do
    {
        //std::cout << "Adding [" << s->x << ", " << s->y << "] to the list" << std::endl;
        draft_path.push_back(s);
        s = s->backpointer;
    }while(s != nullptr);
    if(draft_path.size() == 1)
    {
        throw dstar_exception("The draft path list is empty, check target position");
        return draft_path;
    }
    std::cout << "Constructed a draft path from " << draft_path.size() << " nodes" << std::endl;
    std::cout << "Reduction..." << std::endl;
    state_point* current_viewpoint = origin;
    state_point* prev_visible = origin->backpointer;
    if(prev_visible == nullptr)
    {
        throw dstar_exception("Nothing to reduce, check target position");
        return draft_path;
    }
    state_point* last_visible = origin->backpointer->backpointer;
    if(last_visible == nullptr)
    {
        throw dstar_exception("Nothing to reduce, check target position");
        return draft_path;
    }
    reduced_path.push_back(origin);
    while(last_visible != destination)
    {
        std::pair<double, double> current_coord = current_viewpoint->float_coords();
        std::pair<double, double> proposed_coord = last_visible->float_coords();
        std::pair<double, double> prev_coord = prev_visible->float_coords();
        double current_x = current_coord.first;
        double current_y = current_coord.second;
        double proposed_x = proposed_coord.first;
        double proposed_y = proposed_coord.second;
        double prev_x = prev_coord.first;
        double prev_y = prev_coord.second;
        double hypo = std::sqrt(std::pow(current_x - proposed_x, 2) + std::pow(current_y - proposed_y, 2));
        double cos_theta = (proposed_x - current_x) / hypo;
        double sin_theta = (proposed_y - current_y) / hypo;
        bool ray_traced = true;
        while(std::sqrt(std::pow(current_x - proposed_x, 2) + std::pow(current_y - proposed_y, 2)) >= 1)
        {
            current_x += cos_theta;
            current_y += sin_theta;
            //std::cout << "Checking [" << static_cast<int>(current_x) << ", " << static_cast<int>(current_y) << "]";
            if((std::sqrt(std::pow(current_x - prev_x, 2) + std::pow(current_y - prev_y, 2)) > cutoff_distance) || (map->point(static_cast<int>(current_x), static_cast<int>(current_y))->tag == state_point::stObstacle))
            {
                ray_traced = false;
                break;
            }
        }
        if(!ray_traced)
        {
            reduced_path.push_back(prev_visible);
            //std::cout << "Traced from [" << current_viewpoint->x << ", " << current_viewpoint->y << "] to [" << prev_visible->x << ", " << prev_visible->y << "]" << std::endl;
            current_viewpoint = prev_visible;
        }
        prev_visible = last_visible;
        last_visible = last_visible->backpointer;
    }
    reduced_path.push_back(destination);
    std::cout << "Constructed reduced path from " << reduced_path.size() << " nodes" << std::endl;
    std::cout << "Optimizing..." << std::endl;
    auto current_point_iterator = std::next(reduced_path.begin());
    state_point* current = (*current_point_iterator);
    optimized_path.push_back(origin);
    while(current_point_iterator != reduced_path.end())
    {
        std::vector<state_point*> free(std::pow(r_field, 2));
        std::vector<double> potentials(std::pow(r_field, 2));
        std::list<state_point*> occupied;
        free.clear();
        occupied.clear();
        potentials.clear();
        //std::cout << "Checking [" << current->x << ", " << current->y << "]" << std::endl;
        // 1st pass: division by obstacles and free space
        for(long i = -r_field; i <= r_field; i++)
            for(long j = -r_field; j <= r_field; j++)
            {
                long x_index = current->x + i;
                long y_index = current->y + j;
                //std::cout << "Neighbor [" << x_index << ", " << y_index << "]" << std::endl;
                if((x_index >= 0) &&
                        (x_index < map->get_width()) &&
                        (y_index >= 0) &&
                        (y_index < map->get_height()))
                {
                    if(((i * i) + (j * j)) <= (r_field * r_field))
                    {
                        if(map->point(x_index, y_index)->tag == state_point::stObstacle)
                            occupied.push_back(map->point(x_index, y_index));
                        else
                            free.push_back(map->point(x_index, y_index));
                    }
                }
            }
        //std::cout << "Division: free " << free.size() << ", occupied " << occupied.size() << " on " << r_field << " x " << r_field << " field" << std::endl;
        // 2nd pass: repulsion potential
        if(!occupied.empty())
        {
            for(auto i = free.begin(); i != free.end(); i++)
            {
                state_point* point = (*i);
                double repulsion_potential = 0;
                for(auto o = occupied.begin(); o != occupied.end(); o++)
                {
                    state_point* obstacle = (*o);
                    std::pair<double, double> point_coord = point->float_coords();
                    std::pair<double, double> obstacle_coord = obstacle->float_coords();
                    double point_x = point_coord.first;
                    double point_y = point_coord.second;
                    double obstacle_x = obstacle_coord.first;
                    double obstacle_y = obstacle_coord.second;
                    double sqr_repulsion_distance = std::pow(obstacle_x - point_x, 2) + std::pow(obstacle_y - point_y, 2);
                    repulsion_potential += repulsion_gain / sqr_repulsion_distance;
                }
                potentials.push_back(repulsion_potential);
            }
            auto min_potential_iterator = std::min_element(potentials.begin(), potentials.end());
            size_t min_potential_index = std::distance(potentials.begin(), min_potential_iterator);
            state_point* minimal_point = free.at(min_potential_index);
            //std::cout << "Exchanged [" << current->x << ", " << current->y << "] with (" << minimal_point->x << " " << minimal_point->y << ") with minimal potential " << potentials.at(min_potential_index) << std::endl;
            optimized_path.push_back(minimal_point);
        }
        //current = current->b;
        current = *(++current_point_iterator);
    }
    optimized_path.push_back(destination);
    std::cout << "Constructed optimized path from "
              << optimized_path.size()
              << " nodes, reduction ratio "
              << 1.0 - (static_cast<double>(reduced_path.size()) / static_cast<double>(draft_path.size()))
              << std::endl;
    return optimized_path;
}

dstar::dstar(state_map *_map):
    open_list()
{
    map = _map;
    origin = nullptr;
    destination = nullptr;
    r_field = 5;
    repulsion_gain = 20;
    cutoff_distance = 10;
}

void dstar::init_targets(const long &origin_x,
                         const long &origin_y,
                         const long &destination_x,
                         const long &destination_y,
                         const bool &weighted)
{
    fprintf (stderr, "init target... map(%ld, %ld)\n", map->get_width(), map->get_width());
    if((origin_x < map->get_width()) && (origin_y < map->get_width()) &&
            (destination_x < map->get_width()) && (destination_y < map->get_height()))
    {
        fprintf (stderr, "initialization init target!\n");
        if(destination != nullptr)
            fprintf (stderr, "Destination is not a null pointer!\n");
            reset();
        destination = map->point(destination_x, destination_y);
        destination->cost_actual = 0;
        destination->cost_previous = 0;
        destination->backpointer = nullptr;
        destination->tag = state_point::stOpen;
        open_list.push_back(destination);
        if(origin != nullptr)
        {
            fprintf (stderr, "Origin is not a null pointer!\n");
            origin->weight = origin->weight_previous;
        }
        origin = map->point(origin_x, origin_y);
        for(long i = 0; i < map->get_width(); i++){
            // fprintf (stderr, "filling the colum %ld!\n", i);
            for(long j = 0; j < map->get_height(); j++)
            {
                if((map->point(i, j) != origin) && ((map->point(i, j) != destination)))
                {
                    // fprintf (stderr, "filling the map!\n");
                    std::pair<double, double> origin_coords = origin->float_coords();
                    std::pair<double, double> destination_coords = destination->float_coords();
                    double dist = std::sqrt(std::pow(std::abs(destination_coords.first - origin_coords.first), 2) + std::pow(std::abs(destination_coords.second - origin_coords.second), 2));
                    double dist_dest = std::sqrt(std::pow(std::abs(i - destination_coords.first), 2) + std::pow(std::abs(j - destination_coords.second), 2));
                    double dot_origin = ((i - destination_coords.first) * (origin_coords.first - destination_coords.first)) + ((j - destination_coords.second) * (origin_coords.second - destination_coords.second));
                    double projection = dot_origin / dist;
                    double cost_addition = dist_dest + std::sqrt(std::pow(dist_dest, 2) - std::pow(projection, 2));
                    long l_cost_addition = static_cast<long>(cost_addition);
                    if(weighted)
                        map->point(i,j)->weight += ((l_cost_addition < 0) ? 10 : l_cost_addition);
                }
            }
        }
    }
}

void dstar::set_cutoff_distance(int value)
{
    cutoff_distance = value;
}

void dstar::set_repulsion_gain(double value)
{
    repulsion_gain = (value > 1.0) ? value : 1.0;
}

void dstar::set_r_field(int value)
{
    r_field = (value > 2) ? value : 2;
}

std::list<state_point *> dstar::generate_trajectory()
{
    if((origin == nullptr) || (destination == nullptr))
        {fprintf (stderr, "missing origin or destination!\n");
        return std::list<state_point*>();
        }
    while(iterate_state() != -1){}
    std::cout << std::endl;
    return trace_path();
}

std::list<state_point *> dstar::replan_trajectory(const long &origin_x, const long &origin_y, const long& cost)
{
    if((origin == nullptr) || (destination == nullptr))
        return std::list<state_point*>();
    std::list<state_point *> result;
    result.clear();
    destination->cost_actual = 0;
    destination->cost_previous = 0;
    destination->backpointer = nullptr;
    destination->tag = state_point::stOpen;
    std::vector<state_point *> neighbors = map->neighbors(origin_x, origin_y);
    if(map->point(origin_x, origin_y)->tag == state_point::stObstacle)
    {
        throw dstar_exception("Attempt to replan for point marked as obstacle!");
        return result;
    }
    origin = map->point(origin_x, origin_y);
    point_modified(origin_x, origin_y, cost);
    while(iterate_state() != -1){}
    std::cout << std::endl;
    return trace_path();
}


dstar_exception::dstar_exception(const std::string &msg):
    message(msg)
{
}

const char *dstar_exception::what() const noexcept
{
    return message.c_str();
}
