#define TESLA_INIT_IMPL // If you have more than one file using the tesla header, only define this in the main one
#include <tesla.hpp>    // The Tesla Header
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <switch.h>
#include "rgltr.h"

#define SET_MAX_FREQ 0
#define MAX_CPU_FREQ 2295000000
#define MAX_GPU_FREQ 1536000000
#define MAX_EMC_FREQ 1996800000

#define IS_BAT_CHARGE_ON ((_batteryChargeInfoFields->unk_x14 >> 8) & 1)
#define BAT_CHARGE_THRESHOLD 80
//#define DEBUG_LOGIC

/* rgltr and clk-test are from ZatchyCatGames#6421 */

static Service g_rgltrSrv;

//NX_GENERATE_SERVICE_GUARD(rgltr);

Result rgltrInitialize(void) {
    if(hosversionBefore(8,0,0))
        return MAKERESULT(Module_Libnx, LibnxError_IncompatSysVer);

    return smGetService(&g_rgltrSrv, "rgltr");
}

void rgltrExit(void) {
    serviceClose(&g_rgltrSrv);
}

Service* rgltrGetServiceSession(void) {
    return &g_rgltrSrv;
}

Result rgltrOpenSession(RgltrSession* session_out, PowerDomainId module_id) {
    const u32 in = module_id;
    return serviceDispatchIn(&g_rgltrSrv, 0, in,
        .out_num_objects = 1,
        .out_objects = &session_out->s,
    );
}

Result rgltrGetPowerModuleNumLimit(u32 *out) {
    return serviceDispatchOut(&g_rgltrSrv, 3, *out);
}

void rgltrCloseSession(RgltrSession* session) {
    serviceClose(&session->s);
}

Result rgltrGetVoltageEnabled(RgltrSession* session, u32 *out) {
    return serviceDispatchOut(&session->s, 2, *out);
}

Result rgltrGetVoltage(RgltrSession* session, u32 *out_volt) {
    return serviceDispatchOut(&session->s, 4, *out_volt);
}

///* Notes VoltageAvg
//
//    Vavg time = 175.8ms x 2^(6+VOLT), default: VOLT = 2 (Vavg time = 45s)
//
///End of Notes

extern "C" bool is_mariko();
extern "C" void clk_check();

typedef struct {
    u32 cpu_out_hz;
    u32 gpu_out_hz;
    u32 emc_out_hz;
    u32 cpu_out_volt;
    u32 gpu_out_volt;
    u32 emc_out_volt;
} ClkFields;

bool is_mariko() {
    u64 hardware_type = 0;

    splInitialize();
    splGetConfig(SplConfigItem_HardwareType, &hardware_type);
    splExit();

    switch(hardware_type) {
        case 0: //Icosa
        case 1: //Copper
            return false;
        case 2: //Hoag
        case 3: //Iowa
        case 4: //Calcio
        case 5: //Aula
            return true;
        default:
            return false;
    }
}

void clk_check(ClkFields *out, bool mariko) {
    int res = 0;

    ClkrstSession clkrstSession;
    RgltrSession rgltrSession;

    res = clkrstInitialize();
    if(R_FAILED(res)) {
        fatalThrow(res);
    }

    res = rgltrInitialize();
    if(R_FAILED(res)) {
        fatalThrow(res);
    }

    rgltrOpenSession(&rgltrSession, mariko ? PcvPowerDomainId_Max77812_Cpu : PcvPowerDomainId_Max77621_Cpu);
    clkrstOpenSession(&clkrstSession, PcvModuleId_CpuBus, 3);
    if constexpr(SET_MAX_FREQ)
        clkrstSetClockRate(&clkrstSession, MAX_CPU_FREQ);
    rgltrGetVoltage(&rgltrSession, &out->cpu_out_volt);
    clkrstGetClockRate(&clkrstSession, &out->cpu_out_hz);
    clkrstCloseSession(&clkrstSession);
    rgltrCloseSession(&rgltrSession);

    rgltrOpenSession(&rgltrSession, mariko ? PcvPowerDomainId_Max77812_Gpu : PcvPowerDomainId_Max77621_Gpu);
    clkrstOpenSession(&clkrstSession, PcvModuleId_GPU, 3);
    if constexpr(SET_MAX_FREQ)
        clkrstSetClockRate(&clkrstSession, MAX_GPU_FREQ);
    rgltrGetVoltage(&rgltrSession, &out->gpu_out_volt);
    clkrstGetClockRate(&clkrstSession, &out->gpu_out_hz);
    clkrstCloseSession(&clkrstSession);
    rgltrCloseSession(&rgltrSession);

    rgltrOpenSession(&rgltrSession, mariko ? PcvPowerDomainId_Max77812_Dram : PcvPowerDomainId_Max77620_Sd1);
    clkrstOpenSession(&clkrstSession, PcvModuleId_EMC, 3);
    if constexpr(SET_MAX_FREQ)
        clkrstSetClockRate(&clkrstSession, MAX_EMC_FREQ);
    rgltrGetVoltage(&rgltrSession, &out->emc_out_volt);
    clkrstGetClockRate(&clkrstSession, &out->emc_out_hz);
    clkrstCloseSession(&clkrstSession);
    rgltrCloseSession(&rgltrSession);

    clkrstExit();
    rgltrExit();
}

typedef enum {
    NoHub  = BIT(0),  //If hub is disconnected
    Rail   = BIT(8),  //At least one Joy-con is charging from rail
    SPDSRC = BIT(12), //OTG
    ACC    = BIT(16)  //Accessory
} BatteryChargeInfoFieldsFlags;

typedef enum {
    NewPDO               = 1, //Received new Power Data Object
    NoPD                 = 2, //No Power Delivery source is detected
    AcceptedRDO          = 3  //Received and accepted Request Data Object
} BatteryChargeInfoFieldsPDControllerState; //BM92T series

const char* strPDControllerState[] = {
    "Received new PDO",
    "No PD Source",
    "Received/Accpted RDO"
};

typedef enum {
    None         = 0,
    PD           = 1,
    TypeC_1500mA = 2,
    TypeC_3000mA = 3,
    DCP          = 4,
    CDP          = 5,
    SDP          = 6,
    Apple_500mA  = 7,
    Apple_1000mA = 8,
    Apple_2000mA = 9
} BatteryChargeInfoFieldsChargerType;

const char* strChargerType[] = {
    "None",
    "PD",
    "USB-C@1.5A",
    "USB-C@3.0A",
    "USB-DCP",
    "USB-CDP",
    "USB-SDP",
    "Apple@0.5A",
    "Apple@1.0A",
    "Apple@2.0A",
};

typedef enum {
    Sink         = 1,
    Source       = 2
} BatteryChargeInfoFieldsPowerRole;

const char* strPowerRole[] = {
    "Sink",
    "Source",
};

typedef struct {
    int32_t InputCurrentLimit;                                  //Input (Sink) current limit in mA
    int32_t VBUSCurrentLimit;                                   //Output (Source/VBUS/OTG) current limit in mA
    int32_t ChargeCurrentLimit;                                 //Battery charging current limit in mA (512mA when Docked, 768mA when BatteryTemperature < 17.0 C)
    int32_t ChargeVoltageLimit;                                 //Battery charging voltage limit in mV (3952mV when BatteryTemperature >= 51.0 C)
    int32_t unk_x10;                                            //Possibly an emum, getting the same value as PowerRole in all tested cases
    int32_t unk_x14;                                            //Possibly flags
    BatteryChargeInfoFieldsPDControllerState PDControllerState; //Power Delivery Controller State
    int32_t BatteryTemperature;                                 //Battery temperature in milli C
    int32_t RawBatteryCharge;                                   //Raw battery charged capacity per cent-mille (i.e. 100% = 100000 pcm)
    int32_t VoltageAvg;                                         //Voltage avg in mV (more in Notes)
    int32_t BatteryAge;                                         //Battery age (capacity full / capacity design) per cent-mille (i.e. 100% = 100000 pcm)
    BatteryChargeInfoFieldsPowerRole PowerRole;
    BatteryChargeInfoFieldsChargerType ChargerType;
    int32_t ChargerVoltageLimit;                                //Charger and external device voltage limit in mV
    int32_t ChargerCurrentLimit;                                //Charger and external device current limit in mA
    BatteryChargeInfoFieldsFlags Flags;                         //Unknown flags
} BatteryChargeInfoFields;

Result psmGetBatteryChargeInfoFields(Service* psmService, BatteryChargeInfoFields *out) {
    return serviceDispatchOut(psmService, 17, *out);
}

static Result psmEnableBatteryCharging(Service* psmService) {
    return serviceDispatch(psmService, 2);
}

static Result psmDisableBatteryCharging(Service* psmService) {
    return serviceDispatch(psmService, 3);
}

static Result psmEnableFastBatteryCharging(Service* psmService) {
    return serviceDispatch(psmService, 10);
}

static Result psmDisableFastBatteryCharging(Service* psmService) {
    return serviceDispatch(psmService, 11);
}

FanController g_ICon;
float rotationSpeedLevel = 0;
bool reduceBatteryAging = false;
bool isFastChargingEnabled = false;
bool isFirstRun = true;
unsigned int changeReduceBatteryAgingState = 0;

void reduceBatteryAgingInitialize()
{
    if(isFirstRun)
    {
        static Service* psmService = psmGetServiceSession();
        psmDisableFastBatteryCharging(psmService); //Since isFastChargingEnabled = false by default

        //In the first run, if battery charging is disabled, then reduceBatteryAging must be true,
        // which can be used to adopt settings if user has exited InfoNX before.
        static BatteryChargeInfoFields* _batteryChargeInfoFields = new BatteryChargeInfoFields;
        psmGetBatteryChargeInfoFields(psmService, _batteryChargeInfoFields);
        if(!reduceBatteryAging && !IS_BAT_CHARGE_ON)
            reduceBatteryAging = true;

        isFirstRun = false;
    }
}

bool threadexit = false;
int32_t socTempMili = 0;
char Print_x[800];
Thread t0;
int logic = 0;
u32 batCharge = 0;
bool IsEnoughPowerSupplied;

void Loop(void*) {
    static Service* psmService = psmGetServiceSession();
    static BatteryChargeInfoFields* _batteryChargeInfoFields = new BatteryChargeInfoFields;
    static ClkFields* _clkFields = new ClkFields;
    while (threadexit == false) {
        if(changeReduceBatteryAgingState)
        {
            changeReduceBatteryAgingState = 0;
            reduceBatteryAging = !reduceBatteryAging;
            logic = -1;
        }
        psmGetBatteryChargeInfoFields(psmService, _batteryChargeInfoFields);
        clk_check(_clkFields, is_mariko());
        tsGetTemperatureMilliC(TsLocation_External, &socTempMili);
        psmGetBatteryChargePercentage(&batCharge);
        psmIsEnoughPowerSupplied(&IsEnoughPowerSupplied);

        //Then check logic, if there's something wrong, then it will adjust to reduceBatteryAging settings.
        if(!reduceBatteryAging && (!IS_BAT_CHARGE_ON || !isFastChargingEnabled))
        {
            //reduceBatteryAging is off, then the other two always set to default.
            psmEnableBatteryCharging(psmService);
            psmEnableFastBatteryCharging(psmService);
            isFastChargingEnabled = true;
            logic = 1;
        }
        else if(!reduceBatteryAging /*&& IS_BAT_CHARGE_ON && isFastChargingEnabled*/)
        {
            //reduceBatteryAging is off and the other two set to default, nothing to do.
            logic = 2;
        }
        else if(reduceBatteryAging && isFastChargingEnabled)
        {
            //reduceBatteryAging is on, then Fast Charging will be always disabled.
            psmDisableFastBatteryCharging(psmService);
            isFastChargingEnabled = false;
            logic = 3;
        }
        else if(batCharge >= BAT_CHARGE_THRESHOLD)
        {
            logic = 40;
            if(IS_BAT_CHARGE_ON)
            {
                //If it's >BAT_CHARGE_THRESHOLD% and still charging, then disable charging.
                psmDisableBatteryCharging(psmService);
                logic = 41;
            }
        }
        else
        {
            logic = 50;
            if(!IS_BAT_CHARGE_ON)
            {
                //Only remains this condition, if it's <=BAT_CHARGE_THRESHOLD% and not charging, then enable it.
                psmEnableBatteryCharging(psmService);
                logic = 51;
            }
        }

        snprintf(Print_x, sizeof(Print_x),
            "Current Limit: %u mA IN, %u mA OUT"
            "\nBattery Charg. Limit: %u mA, %u mV"
            "\nunk_x10: 0x%08" PRIx32
            "\nunk_x14: 0x%08" PRIx32
            "\nPD Contr. State: %s (%u)"
            "\nBattery Temp.: %.2f\u00B0C"
            "\nRaw Battery Charge: %.2f%%"
            "\nVoltage Avg: %u mV"
            "\nBattery Age: %.2f%%"
            "\nPower Role: %s (%u)"
            "\nCharger Type: %s (%u)"
            "\nCharger Limit: %u mV, %u mA"
            "\nunk_x3c: 0x%08" PRIx32
            "\nEnough Power Supplied: %s"
            "\n"
            "\nCPU: %6.1f MHz, %6.1f mV"
            "\nGPU: %6.1f MHz, %6.1f mV"
            "\nEMC: %6.1f MHz, %6.1f mV"
            "\n"
            // "\nCPU Clock: %6.1f MHz"
            // "\nCPU Volt : %6.1f mV"
            // "\nGPU Clock: %6.1f MHz"
            // "\nGPU Volt : %6.1f mV"
            // "\nEMC Clock: %6.1f MHz"
            // "\nEMC Volt : %6.1f mV"
            "\nSoC Temp : %2.2f \u00B0C"
            "\nFan Speed: %2.2f %%"
            "\n"
            "\nReduce Battery Aging Status: %s"
            "\n - Enable Slow Charging(0.5A) (%s)"
            "\n - Disable Charging > %d%% (%s)"
            #ifdef DEBUG_LOGIC
                "\nLogic: %d"
            #endif
            ,
            _batteryChargeInfoFields->InputCurrentLimit, 
            _batteryChargeInfoFields->VBUSCurrentLimit,
            _batteryChargeInfoFields->ChargeCurrentLimit,
            _batteryChargeInfoFields->ChargeVoltageLimit, 
            _batteryChargeInfoFields->unk_x10,
            _batteryChargeInfoFields->unk_x14, 
            strPDControllerState[_batteryChargeInfoFields->PDControllerState], _batteryChargeInfoFields->PDControllerState,
            (float)_batteryChargeInfoFields->BatteryTemperature / 1000, 
            (float)_batteryChargeInfoFields->RawBatteryCharge / 1000,
            _batteryChargeInfoFields->VoltageAvg,
            (float)_batteryChargeInfoFields->BatteryAge / 1000,
            strPowerRole[_batteryChargeInfoFields->PowerRole], _batteryChargeInfoFields->PowerRole,
            strChargerType[_batteryChargeInfoFields->ChargerType], _batteryChargeInfoFields->ChargerType,
            _batteryChargeInfoFields->ChargerVoltageLimit,
            _batteryChargeInfoFields->ChargerCurrentLimit,
            (int32_t)_batteryChargeInfoFields->Flags,
            IsEnoughPowerSupplied ? "Yes" : "No",
            (double)_clkFields->cpu_out_hz / 1000000,
            (double)_clkFields->cpu_out_volt / 1000,
            (double)_clkFields->gpu_out_hz / 1000000,
            (double)_clkFields->gpu_out_volt / 1000,
            (double)_clkFields->emc_out_hz / 1000000,
            (double)_clkFields->emc_out_volt / 1000,
            (float)socTempMili / 1000,
            rotationSpeedLevel * 100,
            reduceBatteryAging ? "ON" : "OFF",
            !isFastChargingEnabled ? "ON" : "OFF",
            BAT_CHARGE_THRESHOLD,
            !IS_BAT_CHARGE_ON ? "ON" : "OFF"
            #ifdef DEBUG_LOGIC
                ,logic
            #endif
        );
        svcSleepThread(1000'000'000);
    }
    delete _batteryChargeInfoFields;
    delete _clkFields;
}

class GuiTest : public tsl::Gui {
public:
    GuiTest(u8 arg1, u8 arg2, bool arg3) { }

    // Called when this Gui gets loaded to create the UI
    // Allocate all elements on the heap. libtesla will make sure to clean them up when not needed anymore
    virtual tsl::elm::Element* createUI() override {
        // A OverlayFrame is the base element every overlay consists of. This will draw the default Title and Subtitle.
        // If you need more information in the header or want to change it's look, use a HeaderOverlayFrame.
        auto frame = new tsl::elm::OverlayFrame("InfoNX", APP_VERSION);

        // A list that can contain sub elements and handles scrolling
        auto list = new tsl::elm::List();

        auto *batteryAgingToggle = new tsl::elm::ToggleListItem("Reduce battery aging", reduceBatteryAging);
        batteryAgingToggle->setClickListener([](u64 keys) {
            if (keys & HidNpadButton_A) {
                changeReduceBatteryAgingState++;
                return true;
            }
            return false;
        });

        list->addItem(batteryAgingToggle);
        
        list->addItem(new tsl::elm::CustomDrawer([](tsl::gfx::Renderer *renderer, s32 x, s32 y, s32 w, s32 h) {
            renderer->drawString(Print_x, false, x, y + 30, 16, renderer->a(0xFFFF));
        }), 420);

        // Add the list to the frame for it to be drawn
        frame->setContent(list);
        
        // Return the frame to have it become the top level element of this Gui
        return frame;
    }

    // Called once every frame to update values
    virtual void update() override {}

    // Called once every frame to handle inputs not handled by other UI elements
    virtual bool handleInput(u64 keysDown, u64 keysHeld, const HidTouchState &touchPos, HidAnalogStickState joyStickPosLeft, HidAnalogStickState joyStickPosRight) override {
        return false;   // Return true here to singal the inputs have been consumed
    }
};

class OverlayTest : public tsl::Overlay {
public:
    // libtesla already initialized fs, hid, pl, pmdmnt, hid:sys and set:sys
    virtual void initServices() override {
        smInitialize();
        psmInitialize();
        tsInitialize();
        fanInitialize();
        fanOpenController(&g_ICon, 0x3D000001);
        reduceBatteryAgingInitialize();
        threadCreate(&t0, Loop, NULL, NULL, 0x4000, 0x3F, -2);
        threadStart(&t0);
    }  // Called at the start to initialize all services necessary for this Overlay
    
    virtual void exitServices() override {
        threadexit = true;
        threadWaitForExit(&t0);
        threadClose(&t0);
        fanControllerClose(&g_ICon);
        fanExit();
        tsExit();
        psmExit();
        smExit();
    }  // Callet at the end to clean up all services previously initialized

    virtual void onShow() override {}    // Called before overlay wants to change from invisible to visible state
    
    virtual void onHide() override {}    // Called before overlay wants to change from visible to invisible state

    virtual std::unique_ptr<tsl::Gui> loadInitialGui() override {
        return initially<GuiTest>(1, 2, true);  // Initial Gui to load. It's possible to pass arguments to it's constructor like this
    }
};

int main(int argc, char **argv) {
    return tsl::loop<OverlayTest>(argc, argv);
}
