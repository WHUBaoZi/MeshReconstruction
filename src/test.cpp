//#include <omp.h>
//#include <CGAL/Polygon_mesh_processing/random_perturbation.h>
//#include <openvdb/openvdb.h>
//#include <windows.h>
//
//#include "Remesh.h"
//#include "UtilLib.h"
//#include "CGALTypes.h"
//
//
//int main(int argc, char* argv[])
//{
//    STARTUPINFO si = { 0 };
//    si.cb = sizeof(si);
//    PROCESS_INFORMATION pi;
//
//    std::string inputMeshPath = "D:/DATA/AcademicRelevance/MeshReconstruction/MeshReconstruction/MeshReconstruction/Data/Output/test6_HoleFill/ExtendedMarchingCubeMesh.off";
//    std::string outputMeshPath = "D:/DATA/AcademicRelevance/MeshReconstruction/MeshReconstruction/MeshReconstruction/Data/Output/test6_HoleFill/ExtendedMarchingCubeMesh_fixed.off";
//
//    Mesh mesh;
//    CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(outputMeshPath, mesh);
//
//    //std::string cmd = "MeshFix.exe \"" + inputMeshPath + "\" \"" + outputMeshPath + "\"";
//
//    //// CreateProcess 要求 lpCommandLine 是可写的 char*，所以要用数组
//    //char cmdLine[1024];
//    //strcpy_s(cmdLine, cmd.c_str());
//
//    //if (CreateProcess(
//    //    NULL,        // lpApplicationName 可以为 NULL
//    //    cmdLine,     // lpCommandLine
//    //    NULL, NULL, FALSE, 0, NULL, NULL,
//    //    &si, &pi))
//    //{
//    //    WaitForSingleObject(pi.hProcess, INFINITE);
//    //    CloseHandle(pi.hProcess);
//    //    CloseHandle(pi.hThread);
//    //    std::cout << "MeshFix executed successfully" << std::endl;
//    //}
//    //else
//    //{
//    //    DWORD error = GetLastError();
//    //    std::cerr << "MeshFix activate fail, error code: " << error << std::endl;
//    //}
//}