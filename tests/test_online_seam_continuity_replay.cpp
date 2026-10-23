/** @file test_online_seam_continuity_replay.cpp @brief 冻结同帧日志上的连续输出策略复核，非闭环真值回放。 */
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include "track/OnlineSeamContinuity.h"
namespace v = i2r_rlc::vision;
std::vector<std::string> split(const std::string &s)
{
    std::vector<std::string> out; std::istringstream in(s); std::string x;
    while(std::getline(in,x,'\t')) out.push_back(x);
    return out;
}
int main()
{
    namespace fs=std::filesystem;
    const fs::path root(I2R_RLC_PROJECT_SOURCE_DIR);
    fs::path recording;
    const auto recordings = root / "output/online_correction";
    if (fs::exists(recordings)) {
        for (const auto &entry : fs::directory_iterator(recordings)) {
            if (entry.is_directory()
                && entry.path().filename().wstring().find(L"20261021_224020_914") == 0) {
                if (!recording.empty()) { std::cerr << "Ambiguous incident recording\n"; return 1; }
                recording = entry.path() / "vision_diagnostics.tsv";
            }
        }
    }
    std::ifstream in(recording);
    if(!in) { std::cout<<"SKIP: field recording unavailable\n";return 77; }
    std::string line;std::getline(in,line);auto heads=split(line);
    std::map<std::string,std::size_t> columns;
    for(std::size_t i=0;i<heads.size();++i) columns[heads[i]]=i;
    std::ofstream out(root / "output/analysis/kcf_seam_continuity_20261021/native_policy_replay.csv");
    if(!out) return 1;
    out<<"frame,previous_valid,new_valid,source,degraded_frames,transition_limited,x,y,independent_geometry_available\n";
    v::COnlineSeamContinuity policy;v::KcfFusionOptions fusion;
    int count=0,oldInvalid=0,newInvalid=0;std::map<int,int> sources;
    while(std::getline(in,line)) {
        const auto row=split(line);
        const auto number=[&](const std::string &name) {
            const auto &s=row.at(columns.at(name));return s.empty()?0.0:std::stod(s);
        };
        v::CKcfUpdateDiagnostic t;
        t.completed=number("tracker_completed")!=0; t.hasResponse=number("has_response")!=0;
        t.correlationJumpRejected=number("correlation_jump_rejected")!=0;
        t.fusionJumpRejected=number("fusion_jump_rejected")!=0;
        t.responsePeak=number("response_peak");t.fusionApplied=number("fusion_applied")!=0;
        t.gainX=number("kcf_weight_x");t.gainY=number("kcf_weight_y");
        v::OnlineSeamRefinementResult g;g.valid=number("geometry_valid")!=0;
        g.point={number("geometry_x_px"),number("geometry_y_px")};
        // 日志没有原图对比度；仅复核已有粗跟踪门禁，不能证明烟雾帧的可观测性。
        const bool usable=t.completed&&t.hasResponse&&!t.correlationJumpRejected
            &&!t.fusionJumpRejected&&std::isfinite(t.responsePeak)&&t.responsePeak>=.45;
        const auto r=policy.update({static_cast<int>(number("image_width_px")),static_cast<int>(number("image_height_px"))},
            {number("sim_x_px"),number("sim_y_px")},
            {number("before_fusion_x_px"),number("before_fusion_y_px")},usable,t,g,fusion);
        const bool old=number("result_valid")!=0;++count;oldInvalid+=!old;newInvalid+=!r.valid;
        ++sources[static_cast<int>(r.source)];
        out<<number("frame_id")<<','<<old<<','<<r.valid<<','<<static_cast<int>(r.source)<<','
            <<r.degradedFrames<<','<<r.transitionLimited<<','<<r.point.x<<','<<r.point.y<<','<<g.valid<<'\n';
    }
    std::cout<<"rows="<<count<<" old_invalid="<<oldInvalid<<" new_invalid="<<newInvalid<<'\n';
    for(const auto &s:sources)std::cout<<"source="<<s.first<<" count="<<s.second<<'\n';
    std::cout<<"Frozen observation policy replay only; no new KCF/ICP/robot closed loop or physical accuracy proof.\n";
    return count==7464&&oldInvalid==2397&&newInvalid==0?0:1;
}
