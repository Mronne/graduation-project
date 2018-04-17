
#define CGAL_EIGEN3_ENABLED
#include <OpenMesh\/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/graph_traits_PolyMesh_ArrayKernelT.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/boost/graph/helpers.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/foreach.hpp>

typedef OpenMesh::PolyMesh_ArrayKernelT<> Mesh;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
int main()
{

  Mesh mesh;
  const char* filename = "result_2.obj";
  std::cout<<1<<std::endl;
  OpenMesh::IO::read_mesh(mesh, filename);

  // Incrementally fill the holes
  unsigned int nb_holes = 0;
  BOOST_FOREACH(halfedge_descriptor h, halfedges(mesh))
  {
    
	  if(CGAL::is_border(h,mesh))
    {
      
		std::vector<face_descriptor>  patch_facets;
      std::vector<vertex_descriptor> patch_vertices;
 CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                  mesh,
                  h,
                  std::back_inserter(patch_facets),
                  std::back_inserter(patch_vertices),
      CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, mesh)).geom_traits(Kernel())
				  );    
	  std::cout<<"Ö´ÐÐ"<<std::endl;
      CGAL_assertion(CGAL::is_valid_polygon_mesh(mesh));

      std::cout << "* FILL HOLE NUMBER " << ++nb_holes << std::endl;
      std::cout << "  Number of facets in constructed patch: " << patch_facets.size() << std::endl;
      std::cout << "  Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
      //std::cout << "  Is fairing successful: " << success << std::endl;
	  
    }
  }

  CGAL_assertion(CGAL::is_valid_polygon_mesh(mesh));
  std::cout << std::endl;
  std::cout << nb_holes << " holes have been filled" << std::endl;

  OpenMesh::IO::write_mesh(mesh, "merge_done2.obj");

	
  return 0;
}

