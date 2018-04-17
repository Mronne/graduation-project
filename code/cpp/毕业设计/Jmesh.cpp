#include "jmesh.h"

int main(int argc,char *argv[])
{
	JMesh::init();
	Triangulation tin;
	Edge *a;
	tin.load("result_1.ply");
	int f = tin.checkAndRepair();
	tin.removeDuplicatedTriangles();
	tin.removeDegenerateTriangles();
	tin.removeSmallestComponents();
	tin.removeOverlappingTriangles();
	tin.mergeCoincidentEdges();
	//tin.StarTriangulateHole(a);
	tin.savePLY("result_2.ply");
}