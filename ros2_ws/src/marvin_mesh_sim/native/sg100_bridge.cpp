// SG100 原版 C++ SDK 的 C ABI 适配层，供 Python ctypes 调用。
// 命令打包和反馈解析仍由原版库完成；句柄由 sg_open 创建并由 sg_close 释放。
// Thin C ABI only: all command packing and feedback decoding stay in vendor SDK.
#include "sg100_device.h"
#include <cstring>
#include <memory>
#include <exception>
static thread_local std::string error;
struct Handle {
    SG100Device device;
    unsigned long long updates=0;
    explicit Handle(bool left):device(left?"SIM_LEFT":"SIM_RIGHT") {
        device.RegisterHandStatusCallback([this](const Hand_Status_t&){++updates;});
    }
};
extern "C" {
const char* sg_error(){return error.c_str();}
void* sg_open(const char* root,int left){
    try {
        std::unique_ptr<Handle> h(new Handle(left));
        auto d=&h->device;
        d->SetConfigSearchDir(root);
        if(!d->SetAngleMapHand(left?SG100AngleMapHand::LEFT:SG100AngleMapHand::RIGHT)
           || !d->ValidateAngleMap() || !d->InitForCANFD() || !d->Start()) {
            error="SG100 angle-map validation or passive CANFD startup failed";return nullptr;
        }
        return h.release();
    }catch(const std::exception& e){error=e.what();return nullptr;}
}
void sg_close(void* p){if(p){auto h=static_cast<Handle*>(p);h->device.Stop();delete h;}}
int sg_set(void* p,const float* q){return static_cast<Handle*>(p)->device.SetJointPositions(q,JOINT_COUNT);}
int sg_set_one(void* p,int id,float q){return static_cast<Handle*>(p)->device.SetJointPosition(id,q);}
int sg_status(void* p){return static_cast<Handle*>(p)->device.RequestHandStatus();}
int sg_tx(void* p,uint32_t* id,uint8_t* data){
    CANFD_Frame_t f{};
    if(!static_cast<Handle*>(p)->device.CANFDGetTxFrame(f))return 0;
    *id=f.can_id;std::memcpy(data,f.data,f.data_len);return f.data_len;
}
int sg_rx(void* p,uint32_t id,const uint8_t* data,int len){
    return static_cast<Handle*>(p)->device.CANFDProcessRxFrame(id,data,len);
}
unsigned long long sg_updates(void* p){return static_cast<Handle*>(p)->updates;}
void sg_positions(void* p,float* q){static_cast<Handle*>(p)->device.GetCachedPositions(q);}
}
