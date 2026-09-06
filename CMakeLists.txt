cmake_minimum_required(VERSION 4.4)
project(rigidBodyTutorial)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


 
set(collision_HEADERS include/collision/CollisionDetect.h 
                      include/collision/Geometry.h )
set(collision_SOURCE src/collision/CollisionDetect.cpp ) 

set(rigidbody_HEADERS include/rigidbody/RigidBody.h
                      include/rigidbody/RigidBodySystem.h 
                      include/rigidbody/Scenarios.h )
set(rigidbody_SOURCE src/rigidbody/RigidBody.cpp 
                     src/rigidbody/RigidBodySystem.cpp )

set(contact_HEADERS include/contact/Contact.h )
set(contact_SOURCE src/contact/Contact.cpp )

set(solver_HEADERS include/solvers/Solver.h 
                   include/solvers/SolverBoxPGS.h )
set(solver_SOURCE src/solvers/SolverBoxPGS.cpp )

set(util_HEADERS include/util/OBJLoader.h
                 include/util/MeshAssets.h )
set(util_SOURCE src/util/OBJLoader.cpp
                src/util/MeshAssets.cpp )

set(viewer_HEADERS include/viewer/SimViewer.h)
set(viewer_SOURCE src/viewer/SimViewer.cpp)
        

add_subdirectory(deps/polyscope)
add_subdirectory(deps/eigen)
add_subdirectory(deps/tinyobjloader)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


add_executable (rigidBodyTutorial main.cpp
          ${contact_HEADERS} ${contact_SOURCE} 
          ${collision_HEADERS} ${collision_SOURCE}
          ${rigidbody_HEADERS} ${rigidbody_SOURCE}
          ${solver_HEADERS} ${solver_SOURCE}
		  ${util_HEADERS} ${util_SOURCE}
          ${viewer_HEADERS} ${viewer_SOURCE} )


target_link_libraries(rigidBodyTutorial PRIVATE polyscope Eigen3::Eigen tinyobjloader)


file(
    COPY ${PROJECT_SOURCE_DIR}/resources
    DESTINATION ${PROJECT_BINARY_DIR}
)

