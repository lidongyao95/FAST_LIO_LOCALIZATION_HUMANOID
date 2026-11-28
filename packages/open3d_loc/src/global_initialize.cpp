// Independent global search: writes an initial transform only after validation.
#include <open3d/Open3D.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <unordered_set>
#include <map>
#include <cstdint>

using Cloud = open3d::geometry::PointCloud;
namespace reg = open3d::pipelines::registration;
struct Candidate { Eigen::Matrix4d transform; double fitness, rmse; };

static double angle(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b) {
    return Eigen::AngleAxisd(a.block<3,3>(0,0).transpose()*b.block<3,3>(0,0)).angle();
}
static bool separated(const Candidate &a, const Candidate &b) {
    return (a.transform.block<3,1>(0,3)-b.transform.block<3,1>(0,3)).norm()>.6 ||
           angle(a.transform,b.transform)>10*M_PI/180;
}
static std::shared_ptr<Cloud> read(const std::string &path, double voxel) {
    auto cloud=std::make_shared<Cloud>();
    if (!open3d::io::ReadPointCloud(path,*cloud) || cloud->IsEmpty())
        throw std::runtime_error("cannot read point cloud: "+path);
    cloud->RemoveNonFinitePoints();
    cloud=cloud->VoxelDownSample(voxel);
    if (cloud->points_.size()<100) throw std::runtime_error("insufficient cloud structure");
    cloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel*2.5,40));
    return cloud;
}
static Candidate refine(const Cloud &source,const Cloud &map,Eigen::Matrix4d initial) {
    auto coarse=reg::RegistrationICP(source,map,.8,initial,
        reg::TransformationEstimationPointToPlane(),reg::ICPConvergenceCriteria(1e-6,1e-6,40));
    auto fine=reg::RegistrationICP(source,map,.25,coarse.transformation_,
        reg::TransformationEstimationPointToPlane(),reg::ICPConvergenceCriteria(1e-6,1e-6,40));
    auto score=reg::EvaluateRegistration(source,map,.25,fine.transformation_);
    return {fine.transformation_,score.fitness_,score.inlier_rmse_};
}
static bool quality(const Candidate &c) {
    // Both input frames are gravity-aligned LIO/map frames, not sensor frames.
    return c.transform.allFinite() && c.fitness>=.60 && c.rmse<=.15 &&
        c.transform(2,2)>std::cos(20*M_PI/180);
}
// Gravity-aligned correlative proposals complement sparse/feature-poor scans.
static int64_t cell(int x,int y,int z) {
    return (int64_t(x+1048576)<<42) ^ (int64_t(y+1048576)<<21) ^ int64_t(z+1048576);
}
static double floor_height(const Cloud &cloud) {
    std::map<int,int> histogram;
    for(const auto &p:cloud.points_) histogram[int(std::floor(p.z()/.2))]++;
    auto best=std::max_element(histogram.begin(),histogram.end(),[](const auto&a,const auto&b){return a.second<b.second;});
    return (best->first+.5)*.2;
}
static std::vector<Eigen::Matrix4d> correlate(const Cloud &source,const Cloud &target) {
    const double sfloor=floor_height(source),tfloor=floor_height(target),pitch=1.;
    std::unordered_set<int64_t> occupied;
    for(const auto&p:target.points_) if(p.z()>tfloor+.2 && p.z()<tfloor+3.)
        occupied.insert(cell(int(std::floor(p.x()/pitch)),int(std::floor(p.y()/pitch)),int(std::floor(p.z()/pitch))));
    std::vector<Eigen::Vector3d> structure;
    for(const auto&p:source.points_) if(p.z()>sfloor+.2 && p.z()<sfloor+3.) structure.push_back(p);
    if(structure.size()<30 || occupied.size()<30) return {};
    std::vector<Eigen::Vector3d> samples;
    for(size_t i=0;i<structure.size();i+=std::max(size_t(1),structure.size()/120)) samples.push_back(structure[i]);
    auto lower=target.GetMinBound(),upper=target.GetMaxBound();
    struct Seed { int score; Eigen::Matrix4d transform; };
    std::vector<Seed> top;
    int cutoff=0;
    for(int degrees=0;degrees<360;degrees+=10) {
        const double yaw=degrees*M_PI/180;
        Eigen::Matrix3d rotation=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).matrix();
        std::vector<Eigen::Vector3d> rotated;
        for(const auto&p:samples) rotated.push_back(rotation*p);
        for(double x=lower.x()-2.;x<=upper.x()+2.;x+=1.5)
        for(double y=lower.y()-2.;y<=upper.y()+2.;y+=1.5)
        for(double dz:{-.3,0.,.3}) {
            Eigen::Vector3d translation(x,y,tfloor-sfloor+dz);
            int score=0;
            for(const auto&p:rotated) {
                auto q=p+translation;
                int ix=int(std::floor(q.x()/pitch)),iy=int(std::floor(q.y()/pitch)),iz=int(std::floor(q.z()/pitch));
                if(occupied.count(cell(ix,iy,iz)) || occupied.count(cell(ix+1,iy,iz)) ||
                   occupied.count(cell(ix-1,iy,iz)) || occupied.count(cell(ix,iy+1,iz)) ||
                   occupied.count(cell(ix,iy-1,iz))) score++;
            }
            if(score<cutoff) continue;
            Eigen::Matrix4d transform=Eigen::Matrix4d::Identity();
            transform.block<3,3>(0,0)=rotation; transform.block<3,1>(0,3)=translation;
            top.push_back({score,transform});
            if(top.size()>100) {
                std::sort(top.begin(),top.end(),[](const Seed&a,const Seed&b){return a.score>b.score;});
                top.resize(60);cutoff=top.back().score;
            }
        }
    }
    std::sort(top.begin(),top.end(),[](const Seed&a,const Seed&b){return a.score>b.score;});
    std::vector<Eigen::Matrix4d> result;
    for(const auto &seed:top) {
        bool near=false;
        for(const auto &existing:result) if((seed.transform.block<3,1>(0,3)-existing.block<3,1>(0,3)).norm()<1. &&
            angle(seed.transform,existing)<15*M_PI/180) {near=true;break;}
        if(!near) result.push_back(seed.transform);
        if(result.size()==20) break;
    }
    return result;
}

int main(int argc,char **argv) {
    if(argc!=6) { std::cerr<<"usage: global_initialize MAP SCAN1 SCAN2 SCAN3 OUTPUT.json\n"; return 2; }
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error);
    try {
        auto map=read(argv[1],.12), source=read(argv[2],.12);
        auto coarse_map=map->VoxelDownSample(.4),coarse_source=source->VoxelDownSample(.4);
        coarse_map->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(1.,40));
        coarse_source->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(1.,40));
        auto sf=reg::ComputeFPFHFeature(*coarse_source,open3d::geometry::KDTreeSearchParamHybrid(2.,100));
        std::vector<std::shared_ptr<Cloud>> regions{coarse_map};
        auto mid=coarse_map->GetCenter();
        // Overlapping half-maps force competition beyond the dominant RANSAC mode.
        for(int axis=0;axis<2;axis++) for(int side=0;side<2;side++) {
            std::vector<size_t> ids;
            for(size_t i=0;i<coarse_map->points_.size();i++)
                if(side ? coarse_map->points_[i](axis)>=mid(axis)-2. :
                          coarse_map->points_[i](axis)<=mid(axis)+2.) ids.push_back(i);
            if(ids.size()>=100) regions.push_back(coarse_map->SelectByIndex(ids));
        }
        std::vector<Candidate> candidates;
        unsigned int seed=104;
        for(const auto &region:regions) {
            auto tf=reg::ComputeFPFHFeature(*region,open3d::geometry::KDTreeSearchParamHybrid(2.,100));
            for(int trial=0;trial<3;trial++) {
                reg::CorrespondenceCheckerBasedOnEdgeLength edge(.9);
                reg::CorrespondenceCheckerBasedOnDistance distance(.6);
                std::vector<std::reference_wrapper<const reg::CorrespondenceChecker>> checkers{edge,distance};
                auto proposal=reg::RegistrationRANSACBasedOnFeatureMatching(
                    *coarse_source,*region,*sf,*tf,true,.6,
                    reg::TransformationEstimationPointToPoint(false),3,checkers,
                    reg::RANSACConvergenceCriteria(50000,.999),seed++);
                if(proposal.fitness_<.25) continue;
                auto candidate=refine(*source,*map,proposal.transformation_);
                std::cerr<<"candidate fitness="<<candidate.fitness<<" rmse="<<candidate.rmse<<"\n";
                if(!quality(candidate)) continue;
                bool merged=false;
                for(auto &existing:candidates) if(!separated(existing,candidate)) {
                    if(candidate.fitness>existing.fitness) existing=candidate;
                    merged=true; break;
                }
                if(!merged) candidates.push_back(candidate);
            }
        }
        std::cerr<<"gravity-aligned whole-map correlation search\n";
        for(const auto &proposal:correlate(*coarse_source,*coarse_map)) {
            auto candidate=refine(*source,*map,proposal);
            std::cerr<<"correlation fitness="<<candidate.fitness<<" rmse="<<candidate.rmse<<"\n";
            if(!quality(candidate)) continue;
            bool merged=false;
            for(auto &existing:candidates) if(!separated(existing,candidate)) {
                if(candidate.fitness>existing.fitness) existing=candidate;
                merged=true; break;
            }
            if(!merged) candidates.push_back(candidate);
        }
        if(candidates.empty()) throw std::runtime_error("global search rejected: insufficient overlap or structure");
        std::sort(candidates.begin(),candidates.end(),[](const Candidate&a,const Candidate&b){return a.fitness>b.fitness;});
        const auto best=candidates.front();
        if(candidates.size()>1 && best.fitness-candidates[1].fitness<.08)
            throw std::runtime_error("ambiguous global location: competing separated poses have similar overlap");
        for(int i=3;i<=4;i++) {
            auto validation=read(argv[i],.12);
            auto result=refine(*validation,*map,best.transform);
            std::cerr<<"validation fitness="<<result.fitness<<" rmse="<<result.rmse<<"\n";
            if(!quality(result) || (result.transform.block<3,1>(0,3)-best.transform.block<3,1>(0,3)).norm()>.2 ||
               angle(result.transform,best.transform)>5*M_PI/180)
                throw std::runtime_error("independent scan validation failed: pose not stable");
        }
        // The existing node uses Rx(roll)*Ry(pitch)*Rz(yaw), angles in degrees.
        auto euler=best.transform.block<3,3>(0,0).eulerAngles(0,1,2)*180/M_PI;
        std::ofstream output(argv[5]);
        output<<std::setprecision(12)<<"{\"accepted\":true,\"fitness\":"<<best.fitness
              <<",\"rmse\":"<<best.rmse<<",\"candidate_modes\":"<<candidates.size()<<",\"initial\":[";
        for(int i=0;i<6;i++) { if(i) output<<","; output<<(i<3 ? best.transform(i,3) : euler(i-3)); }
        output<<"],\"matrix\":[";
        for(int i=0;i<4;i++) { if(i) output<<","; output<<"["; for(int j=0;j<4;j++){if(j)output<<",";output<<best.transform(i,j);}output<<"]"; }
        output<<"]}\n";
        if(!output) throw std::runtime_error("cannot write accepted initialization");
        std::cerr<<"global initialization accepted after three scan batches\n";
        return 0;
    } catch(const std::exception &error) { std::cerr<<error.what()<<"\n"; return 2; }
}
