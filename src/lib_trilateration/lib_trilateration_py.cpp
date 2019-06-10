#include "lib_trilateration.h"
#include <boost/python.hpp>

namespace p = boost::python;

p::list toList(Coord3D vector) {
    p::list result = p::list();
    result.append(vector.x);
    result.append(vector.y);
    result.append(vector.z);
    return result;
}

Coord3D fromList(p::list vector) {
    Coord3D result;
    result.x = p::extract<double>(vector[0]);
    result.y = p::extract<double>(vector[1]);
    result.z = p::extract<double>(vector[2]);
    return result;
}

BOOST_PYTHON_MODULE(lib_trilateration_py) {
        def( "get_location", +[](const bool has_4_anchors, const p::list anchor_positions, const p::list ranges) {
            Coord3D best_solution;
            // TODO Switch from C arrays to proper C++ arrays
            // TODO Switch from int to uint for ranges
            // FIXME should not index if not using has_4_anchors
            if(has_4_anchors) {
                Coord3D anchor_positions_array[] = {
                        fromList(p::extract<p::list>(anchor_positions[0])),
                        fromList(p::extract<p::list>(anchor_positions[1])),
                        fromList(p::extract<p::list>(anchor_positions[2])),
                        fromList(p::extract<p::list>(anchor_positions[3]))
                };

                int ranges_array[] = {
                        p::extract<int>(ranges[0]),
                        p::extract<int>(ranges[1]),
                        p::extract<int>(ranges[2]),
                        p::extract<int>(ranges[3])
                };
                int result = lib_trilateration::get_location(&best_solution, has_4_anchors, anchor_positions_array, ranges_array);
                return p::make_tuple(result, toList(best_solution));
            } else {
                Coord3D anchor_positions_array[] = {
                        fromList(p::extract<p::list>(anchor_positions[0])),
                        fromList(p::extract<p::list>(anchor_positions[1])),
                        fromList(p::extract<p::list>(anchor_positions[2])),
                };

                int ranges_array[] = {
                        p::extract<int>(ranges[0]),
                        p::extract<int>(ranges[1]),
                        p::extract<int>(ranges[2]),
                };
                int result = lib_trilateration::get_location(&best_solution, has_4_anchors, anchor_positions_array, ranges_array);
                return p::make_tuple(result, toList(best_solution));
            }
        });
}
