const float adafruit_lsm6dsox_acc[] = {
    0.9988633990287781,
    1.0005004405975342,
    0.9993361830711365,
    -0.003000000026077032,
    -0.003000000026077032,
    0.01900000125169754};

const float motorCalibCoeffs[] = {-2.446e-05, 2.915e-02, 2.611};

const float copterMassKg = 600 * 1e-3; //1200 * 1e-3;

const float gravity = 9.806;

const float L = 0.107;
const float kL = 1.0 / (2.0 * L);

const float ctau = 0.02;
const float kY = 1.0 / (4.0 * ctau);
