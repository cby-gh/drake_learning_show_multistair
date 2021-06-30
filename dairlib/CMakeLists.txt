cmake_minimum_required(VERSION 2.8.3)
project(dairlib)
message("PROJECT_NAME!!!=${PROJECT_NAME}") 
SET(LIBRARY_OUTPUT_PATH "/home/cby/drake_learning/src/drake_learning_show_multistair/lib")
#SET(LIBRARY_OUTPUT_PATH "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/lib")
message("LIBRARY_OUTPUT_PATH=${LIBRARY_OUTPUT_PATH}")
#PROJECT_SOURCE_DIR为CMakeLists.txt所在目录
SET(LIBRARY_OUTPUT_PATH "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/lib")#为什么LIBRARY_OUTPUT_PATH位置不同作用不同,不是位置不同作用不同,是覆盖了
#SET(LIBRARY_OUTPUT_PATH "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/lib")
 set(multibodyutils_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/multibody/multibody_utils.cc")
 set(world_point_evaluator_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/multibody/kinematic/world_point_evaluator.cc")
 set(kinematic_evaluator_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/multibody/kinematic/kinematic_evaluator.cc")
 set(constraint_factory_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/solvers/constraint_factory.cc")
 set(nonlinear_constraint_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/solvers/nonlinear_constraint.cc")
 set(kinematic_evaluator_set_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/multibody/kinematic/kinematic_evaluator_set.cc")
 set(kinematic_constraints_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/multibody/kinematic/kinematic_constraints.cc")
 set(dircon_mode_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/systems/trajectory_optimization/dircon/dircon_mode.cc")
 set(multipose_visualizer_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/multibody/multipose_visualizer.cc")
 set(dynamics_cache_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/systems/trajectory_optimization/dircon/dynamics_cache.cc")
 set(dircon_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/systems/trajectory_optimization/dircon/dircon.cc")
 set(dircon_opt_constraints_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/systems/trajectory_optimization/dircon/dircon_opt_constraints.cc")
 set(find_resource_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/common/find_resource.cc")
 set(spruce_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/third_party/spruce/spruce.cc")
 set(visualization_utils_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/multibody/visualization_utils.cc")
 set(subvector_pass_through_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/systems/primitives/subvector_pass_through.cc")
 set(file_utils_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/common/file_utils.cc")
 set(cassie_utils_path "/home/cby/drake_learning/src/drake_learning_show_multistair/examples/Cassie/cassie_utils.cc")
 set(sim_cassie_sensor_aggregator_path "/home/cby/drake_learning/src/drake_learning_show_multistair/examples/Cassie/sim_cassie_sensor_aggregator.cc")
 set(timestamped_vector_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/systems/framework/timestamped_vector.cc")
 set(output_vector_path "/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/src/systems/framework/output_vector.cc")

message("PROJECT_SOURCE_DIR=${PROJECT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")
message("PROJECT_BINARY_DIR=${PROJECT_BINARY_DIR}")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/multibody")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/multibody/kinematic")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/multibody/solvers")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/systems/trajectory_optimization/dircon")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/common")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/third_party/spruce")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/systems/primitives")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/examples/Cassie")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include/systems/framework")
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("/home/cby/drake_learning/src/drake_learning_show_multistair/dairlib/include/systems/framework")
INCLUDE_DIRECTORIES("/home/cby/drake_learning/src/drake_learning_show_multistair/examples/Cassie")

find_package(drake REQUIRED )#PATHS ${drake_DIR}
message("drake_FOUND=${drake_FOUND}")
message("drake_DIR=${drake_DIR}")
set(ros_drake_LIBRARIES ${drake_LIBRARIES})
message("ros_drake_LIBRARIESxxxxxx=${ros_drake_LIBRARIES}")
include_directories(
  ${ros_drake_LIBRARIES}
)

#ADD_LIBRARY(multibodyutils SHARED ${DAIRlib_src} ) 
ADD_LIBRARY(multibodyutils8 SHARED ${multibodyutils_path} )
target_link_libraries(multibodyutils8
    ${ros_drake_LIBRARIES}
 )

ADD_LIBRARY(kinematicevaluator8 SHARED ${kinematic_evaluator_path} )
target_link_libraries(kinematicevaluator8
    ${ros_drake_LIBRARIES}
 )


ADD_LIBRARY(worldpointevaluator9 SHARED ${world_point_evaluator_path} )
target_link_libraries(worldpointevaluator9
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluator8.so
 )
add_dependencies(worldpointevaluator9 kinematicevaluator8)



ADD_LIBRARY(constraintfactory8 SHARED ${constraint_factory_path} )
target_link_libraries(constraintfactory8
    ${ros_drake_LIBRARIES}
 )

ADD_LIBRARY(kinematicevaluatorset8 SHARED ${kinematic_evaluator_set_path} )
target_link_libraries(kinematicevaluatorset8
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluator8.so
 )
add_dependencies(kinematicevaluatorset8 kinematicevaluator8)

ADD_LIBRARY(dirconmode8 SHARED ${dircon_mode_path} )
target_link_libraries(dirconmode8
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluatorset8.so
 )
add_dependencies(dirconmode8 kinematicevaluatorset8)


ADD_LIBRARY(multiposevisualizer8 SHARED ${multipose_visualizer_path} )
target_link_libraries(multiposevisualizer8
    ${ros_drake_LIBRARIES}
 )


ADD_LIBRARY(dynamicscache8 SHARED ${dynamics_cache_path} )
target_link_libraries(dynamicscache8
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluatorset8.so
 )
add_dependencies(dynamicscache8 kinematicevaluatorset8)

ADD_LIBRARY(dircon8 SHARED ${dircon_path} )
target_link_libraries(dircon8
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libdirconmode8.so 
${PROJECT_SOURCE_DIR}/lib/libdynamicscache8.so 
${PROJECT_SOURCE_DIR}/lib/libmultiposevisualizer8.so
 )
add_dependencies(dircon8 dirconmode8 dynamicscache8 multiposevisualizer8)

ADD_LIBRARY(kinematicconstraints8 SHARED ${kinematic_constraints_path} )
target_link_libraries(kinematicconstraints8
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libnonlinearconstraint8.so
${PROJECT_SOURCE_DIR}/lib/libkinematicevaluatorset8.so
 )
add_dependencies(kinematicconstraints8 nonlinearconstraint8 kinematicevaluatorset8)

ADD_LIBRARY(nonlinearconstraint8 SHARED ${nonlinear_constraint_path} )
target_link_libraries(nonlinearconstraint8
    ${ros_drake_LIBRARIES}
 )

ADD_LIBRARY(dirconoptconstraints8 SHARED ${dircon_opt_constraints_path} )
target_link_libraries(dirconoptconstraints8
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libnonlinearconstraint8.so
${PROJECT_SOURCE_DIR}/lib/libdynamicscache8.so  
${PROJECT_SOURCE_DIR}/lib/libdirconmode8.so   
 )
add_dependencies(dirconoptconstraints8 dirconmode8)

ADD_LIBRARY(spruce8 SHARED ${spruce_path} )
target_link_libraries(spruce8
    ${ros_drake_LIBRARIES}  
 )

ADD_LIBRARY(findresource8 SHARED ${find_resource_path} )
target_link_libraries(findresource8
    ${ros_drake_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/libspruce8.so   
 )
add_dependencies(findresource8 spruce8)
ADD_LIBRARY(subvectorpassthrough8 SHARED ${subvector_pass_through_path} )
target_link_libraries(subvectorpassthrough8
    ${ros_drake_LIBRARIES} 
 )


ADD_LIBRARY(visualizationutils8 SHARED ${visualization_utils_path} )
target_link_libraries(visualizationutils8
    ${ros_drake_LIBRARIES} 
${PROJECT_SOURCE_DIR}/lib/libsubvectorpassthrough8.so
 )
add_dependencies(visualizationutils8 subvectorpassthrough8)

ADD_LIBRARY(fileutils8 SHARED ${file_utils_path} )
target_link_libraries(fileutils8
    ${ros_drake_LIBRARIES} 
 )

ADD_LIBRARY(timestampedvector8 SHARED ${timestamped_vector_path} )
target_link_libraries(timestampedvector8
    ${ros_drake_LIBRARIES} 
 )
##编译cassie project失败，找不到一些hpp文件
##ADD_LIBRARY(simcassiesensoraggregator SHARED ${sim_cassie_sensor_aggregator_path} )
##target_link_libraries(simcassiesensoraggregator
##   ${ros_drake_LIBRARIES} 
##${PROJECT_SOURCE_DIR}/build/lib/libtimestampedvector.so
##${PROJECT_SOURCE_DIR}/build/lib/liboutputvector.so
## )



ADD_LIBRARY(outputvector8 SHARED ${output_vector_path} )
target_link_libraries(outputvector8
    ${ros_drake_LIBRARIES} 
 )

#ADD_LIBRARY(cassieutils SHARED ${cassie_utils_path} )
#target_link_libraries(cassieutils
#    ${ros_drake_LIBRARIES} 
# )


