#include "monitor.h"
#include "mhelpers.h"
#include "adc.h"

#define VBAT_CHAN ADC_Chan_0

#define ADC_SCALE (3.3 / 4096.0)

#define VBAT_BRIDGE_SCALE (12.06 / 2.334)

static struct
{
    RMA_t vbat;
    RMA_t ch1;
    int chanCurr;
} _g_monState =
    {
        .vbat = {.windowSz = (FUSION_FREQ / 10)},
        .ch1 = {.windowSz = (FUSION_FREQ / 10)},
};

/******************************************************************************/

static void _Monitor_ConverVbat(uint32_t vbat);
static void _Monitor_ConverCh1(uint32_t val);

/******************************************************************************/

void Monitor_Update()
{
    ADC_Read(ADC_DEV, _g_monState.chanCurr,
             _g_monState.chanCurr == VBAT_CHAN ? _Monitor_ConverVbat : _Monitor_ConverCh1);
    _g_monState.chanCurr = (_g_monState.chanCurr + 1) % ADC_Chan_Tot;
}

float Monitor_GetVbat()
{
    return _g_monState.vbat.val;
}

float Monitor_GetCh1()
{
    return _g_monState.ch1.val;
}

/******************************************************************************/

static void _Monitor_ConverVbat(uint32_t vbat)
{
    float fvbat = vbat;

    fvbat *= ADC_SCALE;
    fvbat *= VBAT_BRIDGE_SCALE;

    FS_RmaUpdate(&_g_monState.vbat, fvbat);
}

static void _Monitor_ConverCh1(uint32_t val)
{
    float fval = val * ADC_SCALE;
    FS_RmaUpdate(&_g_monState.ch1, fval);
}