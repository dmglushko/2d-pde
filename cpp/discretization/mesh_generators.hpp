#pragma once

#include "geometry.hpp"
#include "triangulate.hpp"

#include <functional>

namespace discretization {

class MeshGenerator {
public:
  virtual ~MeshGenerator()=default;
  virtual MeshAlgorithm algorithm()const=0;
  virtual TriangleMesh createMesh(const Geometry2D& geometry)const {
    MeshOptions configured=options_;
    configured.algorithm=algorithm();
    return Triangulator2d::generate(geometry.createBoundary(),configured);
  }
  void setMaxElementSize(double h){options_.max_element_size=h;}
  void setOptimizationIterations(int n){options_.optimization_iterations=n;}
  void setMaxRefinementPasses(int n){options_.max_refinement_passes=n;}
  void setSizeField(std::function<double(double,double)> field){options_.size_field=std::move(field);}
  double maxElementSize()const{return options_.max_element_size;}
protected:
  MeshOptions options_;
};

class EarClippingMeshGenerator final:public MeshGenerator {
public: MeshAlgorithm algorithm()const override{return MeshAlgorithm::EarClipping;}
};
class QualityEarClippingMeshGenerator final:public MeshGenerator {
public: MeshAlgorithm algorithm()const override{return MeshAlgorithm::QualityEarClipping;}
};
class UniformRefinementMeshGenerator final:public MeshGenerator {
public: MeshAlgorithm algorithm()const override{return MeshAlgorithm::UniformRefinement;}
};
class DelaunayEdgeFlipMeshGenerator final:public MeshGenerator {
public: MeshAlgorithm algorithm()const override{return MeshAlgorithm::DelaunayEdgeFlips;}
};
class LocalSizeRefinementMeshGenerator final:public MeshGenerator {
public: MeshAlgorithm algorithm()const override{return MeshAlgorithm::LocalSizeRefinement;}
};

} // namespace discretization
