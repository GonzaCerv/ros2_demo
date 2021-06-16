// algorithms.h - Prototypes for tsp class, which contains algorithms used
// to solve the traveling salesman problem.
// Written by Sam Best for CS325
// Last modified 3/12/2013

#ifndef _algorithms_h
#define _algorithms_h

#include <signal.h>

#include <deque>
#include <fstream>
#include <vector>

#include "city.h"

// Knapp libraries
#include "knapp_amr/KnappTypes.hpp"

using namespace std;

const char OUTPUT_FN[] = "mysolution.txt";

class tsp {
   public:
    tsp(const char* filename);  // Constructor, takes filename to read from as input
    /**
     * @brief Construct a new tsp object
     *
     * @param parts List of parts that must be search.
     */
    tsp(const std::vector<knapp_amr::Part>& parts);
    tsp(tsp& source);
    ~tsp();
    int read_file(const char* filename);  // Reads a list of cities into original_list from filename
    int read_vector(const std::vector<knapp_amr::Part>& parts);
    void write_solution(const char* file_name);  // Writes a solution to file_name
    /**
     * @brief generates a vector with the solution.
     * 
     * @return std::vector<knapp_amr::Part> 
     */
    std::vector<knapp_amr::Part> generate_solution();
    int brute_force_wrapper();                   // Public function for brute force TSP solution
    std::vector<knapp_amr::Part> find_path();
    int nearest_neighbor();  // Nearest neighbor with 2-opt (calls nearest_neighbor_basic() and
                             // two_change())
    int nearest_neighbor_basic(
        int start_index);         // Generates basic nearest neighbor tour beginning at start_index
    int two_change();             // Semi-naive 2-opt implementation.
    int two_opt();                // Attempt at 2-opt with neighbor lists (NOT WORKING)
    int swap_two(int i, int k);   // Used by two_opt()
    int get_solution_distance();  // Finds total distance of tour in solution
    void display_neighbor_lists();  // Displays neighbor lists for testing
    void fix_positions();           // Updates pos of cities in solution
    void rotate(int pos);           // Used by brute_force(...) to create permutations
   private:
    void brute_force(deque<city*>& best_path, int& min_distance,
                     int cities_left);  // Recursive function called by brute_force_wrapper()
    deque<city*> original_list;         // Stores the initial list of cities
    deque<city*> solution;              // Stores the current solution
    std::vector<knapp_amr::Part> original_parts;
    int num_cities;                     // Stores the number of cities read into original_list
};

void copy_city_deque(deque<city*>& source, deque<city*>& dest);  // Copies a deque of city*
void end_opt(int signum);  // Terminates optimization after SIGTERM signal received
#endif
