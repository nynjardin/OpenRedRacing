#include "scene/comfort_camera.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace bigred;
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double bam = 65536.0/(2*pi);
void check(bool ok, const char* message) {
    if (!ok) { std::fprintf(stderr,"FAIL: %s\n",message); std::exit(1); }
}
double distance(const CameraSample& a, const CameraSample& b) {
    return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
}
double angle(double a, double b) { return std::abs(std::remainder(a-b,65536.0))/bam; }
ComfortTarget trajectory(double t) {
    // A fast flight followed by a continuous quarter turn and ascent.
    if (t<2) return {{10000,-2000,10000+1200*t},0,0,false};
    const double a = std::min((t-2)*pi/4,pi/2);
    return {{10000+1600*(1-std::cos(a)),-2000-100*(t-2),12400+1600*std::sin(a)},a*bam,0,false};
}
CameraSample run(int hz) {
    ComfortCamera rig;
    rig.update(trajectory(0),0,true);
    CameraSample sample;
    for (int i=1; i<=4*hz; ++i) sample=rig.update(trajectory(double(i)/hz),1.0/hz);
    return sample;
}
}

int main() {
    ComfortTarget target{{10000,-2000,10000},0,0,false};
    ComfortCamera hover;
    const CameraSample rest=hover.update(target,0,true);
    CameraSample sample;
    for (int i=1; i<=600; ++i) {
        target.position.y=-2000+40*std::sin(i*pi/30);
        target.yaw=i*800; // turning on the spot must not orbit the eye
        sample=hover.update(target,1.0/120);
        check(distance(sample,rest)<1e-7,"hover bob/body rotation moves the eye");
        check(angle(sample.yaw,rest.yaw)<1e-7,"hover rotation pans the world");
        check(angle(sample.pitch,rest.pitch)<0.002,"small hover bob escapes framing zone");
    }
    for (int hz : {30,60,120,144,240}) {
        const CameraSample reference=run(480), actual=run(hz);
        std::printf("%d Hz: position %.4f, yaw %.4f deg, pitch %.4f deg\n",hz,
                    distance(actual,reference),angle(actual.yaw,reference.yaw)*180/pi,
                    angle(actual.pitch,reference.pitch)*180/pi);
        check(distance(actual,reference)<3,"follow trajectory depends on refresh rate");
        check(angle(actual.yaw,reference.yaw)<pi/180,"aim depends on refresh rate");
        check(angle(actual.pitch,reference.pitch)<pi/180,"pitch depends on refresh rate");
    }
    ComfortCamera turning;
    turning.update(trajectory(0),0,true);
    for (int i=1; i<=480; ++i) {
        const ComfortTarget flight=trajectory(i/120.0);
        sample=turning.update(flight,1.0/120);
        const double dx=flight.position.x-sample.x, dz=flight.position.z-sample.z;
        const double bearing=std::atan2(dx,dz)*bam;
        const double elevation=std::atan2(flight.position.y-80-sample.y,std::hypot(dx,dz))*bam;
        // Comfort allows movement within the frame, but must keep the subject visible
        // even in a 4:3 view (horizontal half-FOV is about 37 degrees).
        check(angle(sample.yaw,bearing)<pi/9,"turn loses the subject horizontally");
        check(angle(sample.pitch,elevation)<pi/9,"ascent loses the subject vertically");
    }
    ComfortCamera reverse;
    target={{10000,-2000,10000},0,0,false};
    reverse.update(target,0,true);
    for (int i=1; i<=240; ++i) {
        target.position.z=10000-75*i; // 9000 units/sec also exercises the lag limit
        sample=reverse.update(target,1.0/120);
        check(angle(sample.yaw,0)<pi/12,"reversing flips the camera");
        check(std::abs(sample.pitch/bam)<pi*4/9+1e-8,"aim approaches vertical singularity");
    }
    // Collision is applied to the final eye; clearing it recovers gently.
    ComfortCamera collision;
    target={{10000,-2000,10000},0,0,false};
    collision.update(target,0,true);
    const auto retract=[](CameraPoint,CameraPoint) {return .4;};
    sample=collision.update(target,1.0/120,false,retract);
    check(std::abs(sample.z-9440)<1e-8,"obstruction does not retract immediately");
    sample=collision.update(target,1.0/120);
    check(sample.z<9440 && sample.z>9400,"boom snaps back after obstruction clears");
    for (int i=0; i<600; ++i) sample=collision.update(target,1.0/120);
    check(distance(sample,rest)<.001,"boom does not recover to full length");
    sample=collision.update(target,1.0/120,false,[](CameraPoint,CameraPoint){return 0.0;});
    const double separation=std::sqrt((sample.x-target.position.x)*(sample.x-target.position.x)+
        (sample.y-(target.position.y-80))*(sample.y-(target.position.y-80))+
        (sample.z-target.position.z)*(sample.z-target.position.z));
    check(std::abs(separation-80)<1e-7,"fully blocked boom collapses eye into focus");
    // Both explicit short cuts and teleport cuts reset all spring state.
    target.position.x+=100;
    sample=collision.update(target,1.0/120,true);
    check(std::abs(sample.x-target.position.x)<1e-8,"short cut retains old position lag");
    target.position.x+=10000;
    sample=collision.update(target,1.0/120);
    check(std::abs(sample.x-target.position.x)<1e-8,"teleport retains old position lag");
    // Brief roof-query flicker must not repeatedly pump the camera height.
    ComfortCamera tunnel;
    target={{10000,-2000,10000},0,0,true};
    const CameraSample inside=tunnel.update(target,0,true);
    target.confined=false;
    for (int i=0;i<24;++i) sample=tunnel.update(target,1.0/120);
    check(distance(sample,inside)<1e-8,"brief tunnel exit expands the boom");
    target.confined=true;
    tunnel.update(target,1.0/120);
    target.confined=false;
    for (int i=0;i<600;++i) sample=tunnel.update(target,1.0/120);
    check(std::abs(sample.z-8600)<.001,"clear tunnel exit never restores normal distance");
    std::puts("Comfort camera checks passed.");
}
