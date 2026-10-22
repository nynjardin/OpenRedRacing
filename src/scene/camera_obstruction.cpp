#include "camera_obstruction.h"
#include "sim/vehicle.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace bigred {
namespace {
const VehicleBox& lens() {
    static const VehicleBox value = [] {
        VehicleBox box;
        box.xmin=box.ytop=box.zmin=-16;
        box.xmax=box.ybottom=box.zmax=16;
        box.radius=24;
        box.sector_reach.fill(24);
        box.build();
        return box;
    }();
    return value;
}
int32_t fixed(double v) {
    return static_cast<int32_t>(std::clamp(v*512,
        double(std::numeric_limits<int32_t>::min()),double(std::numeric_limits<int32_t>::max())));
}
bool blocked(const SimTerrain& terrain, CameraPoint p) {
    const int32_t x=static_cast<int32_t>(p.x), y=static_cast<int32_t>(p.y), z=static_cast<int32_t>(p.z);
    const int32_t ground=terrain.height_fx(fixed(p.x),fixed(p.z))/512;
    if (p.y+16>ground && !camera_under_roof(terrain,p)) return true;

    // Query the same broad phase as vehicle contacts, filtering before narrow
    // phase so a cell full of shrubs cannot crowd a wall out of a contact array.
    const int gx=int(std::floor(p.x/SimTerrain::kGridCell));
    const int gz=int(std::floor(p.z/SimTerrain::kGridCell));
    for (int cz=gz-1;cz<=gz+1;++cz) for (int cx=gx-1;cx<=gx+1;++cx) {
        const auto* cell=terrain.grid_cell(cx,cz);
        if (!cell) continue;
        for (int32_t index : *cell) {
            const SimObject& object=terrain.blockers[static_cast<size_t>(index)];
            if (!(object.hard || object.slab || object.ramp_gate)) continue;
            if (y+16<object.y+object.y_top || y-16>object.y+object.y_bottom) continue;
            if (terrain.blocker_contact(object,x,y,z,0,ground,lens(),false).hit) return true;
        }
    }
    return false;
}
}
bool camera_under_roof(const SimTerrain& terrain, CameraPoint eye) {
    return terrain.valid() && terrain.gather_candidates(fixed(eye.x),fixed(eye.y),fixed(eye.z),
        lens().radius,&lens(),0).inside;
}
double camera_arm_fraction(const SimTerrain& terrain, CameraPoint focus, CameraPoint eye) {
    if (!terrain.valid()) return 1;
    const double dx=eye.x-focus.x, dy=eye.y-focus.y, dz=eye.z-focus.z;
    const double distance=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (distance<1) return 1;
    const int steps=std::clamp(int(std::ceil(distance/24)),1,512);
    for (int i=1;i<=steps;++i) {
        const double t=double(i)/steps;
        if (blocked(terrain,{focus.x+dx*t,focus.y+dy*t,focus.z+dz*t}))
            return std::max(0.0,double(i-1)/steps-16/distance);
    }
    return 1;
}
} // namespace bigred
