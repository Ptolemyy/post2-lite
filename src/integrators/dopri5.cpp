#include "post2/integrators/dopri5.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace post2::integrators {

namespace {

// --- Dormand-Prince 5(4) Butcher tableau ------------------------------------
// c_i, a_ij, b_i (5th-order solution), e_i = b_i - b*_i (embedded error).
constexpr double kC2 = 1.0 / 5.0;
constexpr double kC3 = 3.0 / 10.0;
constexpr double kC4 = 4.0 / 5.0;
constexpr double kC5 = 8.0 / 9.0;
constexpr double kC6 = 1.0;
constexpr double kC7 = 1.0;

constexpr double kA21 = 1.0 / 5.0;

constexpr double kA31 = 3.0 / 40.0;
constexpr double kA32 = 9.0 / 40.0;

constexpr double kA41 = 44.0 / 45.0;
constexpr double kA42 = -56.0 / 15.0;
constexpr double kA43 = 32.0 / 9.0;

constexpr double kA51 = 19372.0 / 6561.0;
constexpr double kA52 = -25360.0 / 2187.0;
constexpr double kA53 = 64448.0 / 6561.0;
constexpr double kA54 = -212.0 / 729.0;

constexpr double kA61 = 9017.0 / 3168.0;
constexpr double kA62 = -355.0 / 33.0;
constexpr double kA63 = 46732.0 / 5247.0;
constexpr double kA64 = 49.0 / 176.0;
constexpr double kA65 = -5103.0 / 18656.0;

constexpr double kA71 = 35.0 / 384.0;
constexpr double kA72 = 0.0;
constexpr double kA73 = 500.0 / 1113.0;
constexpr double kA74 = 125.0 / 192.0;
constexpr double kA75 = -2187.0 / 6784.0;
constexpr double kA76 = 11.0 / 84.0;

// 5th-order solution coefficients (= a7j).
constexpr double kB1 = kA71;
constexpr double kB3 = kA73;
constexpr double kB4 = kA74;
constexpr double kB5 = kA75;
constexpr double kB6 = kA76;
constexpr double kB7 = 0.0;

// Error coefficients e_i = b_i - b*_i, where b*_i is the 4th-order solution.
constexpr double kE1 = 71.0 / 57600.0;
constexpr double kE3 = -71.0 / 16695.0;
constexpr double kE4 = 71.0 / 1920.0;
constexpr double kE5 = -17253.0 / 339200.0;
constexpr double kE6 = 22.0 / 525.0;
constexpr double kE7 = -1.0 / 40.0;

constexpr double kSafetyFactor = 0.9;
constexpr double kFacMin = 0.2;
constexpr double kFacMax = 5.0;
constexpr double kOrderInv = 1.0 / 5.0;

// Dormand-Prince RK45 quartic dense-output matrix. Interpolation is:
// y(theta) = y0 + h * sum_j theta^(j+1) * sum_i P[i][j] * k_i.
constexpr std::array<std::array<double, 4>, 7> kDopri5DenseP{{
    {{1.0,
      -8048581381.0 / 2820520608.0,
      8663915743.0 / 2820520608.0,
      -12715105075.0 / 11282082432.0}},
    {{0.0, 0.0, 0.0, 0.0}},
    {{0.0,
      131558114200.0 / 32700410799.0,
      -68118460800.0 / 10900136933.0,
      87487479700.0 / 32700410799.0}},
    {{0.0,
      -1754552775.0 / 470086768.0,
      14199869525.0 / 1410260304.0,
      -10690763975.0 / 1880347072.0}},
    {{0.0,
      127303824393.0 / 49829197408.0,
      -318862633887.0 / 49829197408.0,
      701980252875.0 / 199316789632.0}},
    {{0.0,
      -282668133.0 / 205662961.0,
      2019193451.0 / 616988883.0,
      -1453857185.0 / 822651844.0}},
    {{0.0,
      40617522.0 / 29380423.0,
      -110615467.0 / 29380423.0,
      69997945.0 / 29380423.0}},
}};

constexpr int kDop853Stages = 12;
constexpr int kDop853StagesExtended = 16;
constexpr int kDop853InterpolatorPower = 7;
constexpr double kDop853OrderInv = 1.0 / 8.0;

constexpr std::array<double, kDop853StagesExtended> kDop853C{{
    0.0,
    0.526001519587677318785587544488e-01,
    0.789002279381515978178381316732e-01,
    0.118350341907227396726757197510,
    0.281649658092772603273242802490,
    0.333333333333333333333333333333,
    0.25,
    0.307692307692307692307692307692,
    0.651282051282051282051282051282,
    0.6,
    0.857142857142857142857142857142,
    1.0,
    1.0,
    0.1,
    0.2,
    0.777777777777777777777777777778,
}};

constexpr std::array<std::array<double, kDop853StagesExtended>, kDop853StagesExtended> kDop853A{{
    {{0.0}},
    {{5.26001519587677318785587544488e-2}},
    {{1.97250569845378994544595329183e-2, 5.91751709536136983633785987549e-2}},
    {{2.95875854768068491816892993775e-2, 0.0, 8.87627564304205475450678981324e-2}},
    {{2.41365134159266685502369798665e-1, 0.0, -8.84549479328286085344864962717e-1,
      9.24834003261792003115737966543e-1}},
    {{3.7037037037037037037037037037e-2, 0.0, 0.0, 1.70828608729473871279604482173e-1,
      1.25467687566822425016691814123e-1}},
    {{3.7109375e-2, 0.0, 0.0, 1.70252211019544039314978060272e-1,
      6.02165389804559606850219397283e-2, -1.7578125e-2}},
    {{3.70920001185047927108779319836e-2, 0.0, 0.0, 1.70383925712239993810214054705e-1,
      1.07262030446373284651809199168e-1, -1.53194377486244017527936158236e-2,
      8.27378916381402288758473766002e-3}},
    {{6.24110958716075717114429577812e-1, 0.0, 0.0, -3.36089262944694129406857109825,
      -8.68219346841726006818189891453e-1, 2.75920996994467083049415600797e1,
      2.01540675504778934086186788979e1, -4.34898841810699588477366255144e1}},
    {{4.77662536438264365890433908527e-1, 0.0, 0.0, -2.48811461997166764192642586468,
      -5.90290826836842996371446475743e-1, 2.12300514481811942347288949897e1,
      1.52792336328824235832596922938e1, -3.32882109689848629194453265587e1,
      -2.03312017085086261358222928593e-2}},
    {{-9.3714243008598732571704021658e-1, 0.0, 0.0, 5.18637242884406370830023853209,
      1.09143734899672957818500254654, -8.14978701074692612513997267357,
      -1.85200656599969598641566180701e1, 2.27394870993505042818970056734e1,
      2.49360555267965238987089396762, -3.0467644718982195003823669022}},
    {{2.27331014751653820792359768449, 0.0, 0.0, -1.05344954667372501984066689879e1,
      -2.00087205822486249909675718444, -1.79589318631187989172765950534e1,
      2.79488845294199600508499808837e1, -2.85899827713502369474065508674,
      -8.87285693353062954433549289258, 1.23605671757943030647266201528e1,
      6.43392746015763530355970484046e-1}},
    {{5.42937341165687622380535766363e-2, 0.0, 0.0, 0.0, 0.0,
      4.45031289275240888144113950566, 1.89151789931450038304281599044,
      -5.8012039600105847814672114227, 3.1116436695781989440891606237e-1,
      -1.52160949662516078556178806805e-1, 2.01365400804030348374776537501e-1,
      4.47106157277725905176885569043e-2}},
    {{5.61675022830479523392909219681e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
      2.53500210216624811088794765333e-1, -2.46239037470802489917441475441e-1,
      -1.24191423263816360469010140626e-1, 1.5329179827876569731206322685e-1,
      8.20105229563468988491666602057e-3, 7.56789766054569976138603589584e-3,
      -8.298e-3}},
    {{3.18346481635021405060768473261e-2, 0.0, 0.0, 0.0, 0.0,
      2.83009096723667755288322961402e-2, 5.35419883074385676223797384372e-2,
      -5.49237485713909884646569340306e-2, 0.0, 0.0,
      -1.08347328697249322858509316994e-4, 3.82571090835658412954920192323e-4,
      -3.40465008687404560802977114492e-4, 1.41312443674632500278074618366e-1}},
    {{-4.28896301583791923408573538692e-1, 0.0, 0.0, 0.0, 0.0,
      -4.69762141536116384314449447206, 7.68342119606259904184240953878,
      4.06898981839711007970213554331, 3.56727187455281109270669543021e-1,
      0.0, 0.0, 0.0, -1.39902416515901462129418009734e-3,
      2.9475147891527723389556272149, -9.15095847217987001081870187138}},
}};

constexpr std::array<double, kDop853Stages> kDop853B{{
    5.42937341165687622380535766363e-2,
    0.0,
    0.0,
    0.0,
    0.0,
    4.45031289275240888144113950566,
    1.89151789931450038304281599044,
    -5.8012039600105847814672114227,
    3.1116436695781989440891606237e-1,
    -1.52160949662516078556178806805e-1,
    2.01365400804030348374776537501e-1,
    4.47106157277725905176885569043e-2,
}};

constexpr std::array<double, kDop853Stages + 1> kDop853E3{{
    5.42937341165687622380535766363e-2 - 0.244094488188976377952755905512,
    0.0,
    0.0,
    0.0,
    0.0,
    4.45031289275240888144113950566,
    1.89151789931450038304281599044,
    -5.8012039600105847814672114227,
    3.1116436695781989440891606237e-1 - 0.733846688281611857341361741547,
    -1.52160949662516078556178806805e-1,
    2.01365400804030348374776537501e-1,
    4.47106157277725905176885569043e-2 - 0.220588235294117647058823529412e-1,
    0.0,
}};

constexpr std::array<double, kDop853Stages + 1> kDop853E5{{
    0.1312004499419488073250102996e-1,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.1225156446376204440720569753e+1,
    -0.4957589496572501915214079952,
    0.1664377182454986536961530415e+1,
    -0.3503288487499736816886487290,
    0.3341791187130174790297318841,
    0.8192320648511571246570742613e-1,
    -0.2235530786388629525884427845e-1,
    0.0,
}};

constexpr std::array<std::array<double, kDop853StagesExtended>, kDop853InterpolatorPower - 3> kDop853D{{
    {{-0.84289382761090128651353491142e+1, 0.0, 0.0, 0.0, 0.0,
      0.56671495351937776962531783590, -0.30689499459498916912797304727e+1,
      0.23846676565120698287728149680e+1, 0.21170345824450282767155149946e+1,
      -0.87139158377797299206789907490, 0.22404374302607882758541771650e+1,
      0.63157877876946881815570249290, -0.88990336451333310820698117400e-1,
      0.18148505520854727256656404962e+2, -0.91946323924783554000451984436e+1,
      -0.44360363875948939664310572000e+1}},
    {{0.10427508642579134603413151009e+2, 0.0, 0.0, 0.0, 0.0,
      0.24228349177525818288430175319e+3, 0.16520045171727028198505394887e+3,
      -0.37454675472269020279518312152e+3, -0.22113666853125306036270938578e+2,
      0.77334326684722638389603898808e+1, -0.30674084731089398182061213626e+2,
      -0.93321305264302278729567221706e+1, 0.15697238121770843886131091075e+2,
      -0.31139403219565177677282850411e+2, -0.93529243588444783865713862664e+1,
      0.35816841486394083752465898540e+2}},
    {{0.19985053242002433820987653617e+2, 0.0, 0.0, 0.0, 0.0,
      -0.38703730874935176555105901742e+3, -0.18917813819516756882830838328e+3,
      0.52780815920542364900561016686e+3, -0.11573902539959630126141871134e+2,
      0.68812326946963000169666922661e+1, -0.10006050966910838403183860980e+1,
      0.77771377980534432092869265740, -0.27782057523535084065932004339e+1,
      -0.60196695231264120758267380846e+2, 0.84320405506677161018159903784e+2,
      0.11992291136182789328035130030e+2}},
    {{-0.25693933462703749003312586129e+2, 0.0, 0.0, 0.0, 0.0,
      -0.15418974869023643374053993627e+3, -0.23152937917604549567536039109e+3,
      0.35763911791061412378285349910e+3, 0.93405324183624310003907691704e+2,
      -0.37458323136451633156875139351e+2, 0.10409964950896230045147246184e+3,
      0.29840293426660503123344363579e+2, -0.43533456590011143754432175058e+2,
      0.96324553959188282948394950600e+2, -0.39177261675615439165231486172e+2,
      -0.14972683625798562581422125276e+3}},
}};

ExtendedState add_state_scaled_derivative(
    const ExtendedState& s, const ExtendedDerivative& d, double scale)
{
    ExtendedState out;
    out.motion.position_m = s.motion.position_m + d.motion_dot.d_position_mps * scale;
    out.motion.velocity_mps = s.motion.velocity_mps + d.motion_dot.d_velocity_mps2 * scale;
    out.tank_masses_kg.resize(s.tank_masses_kg.size());
    const std::size_t n = std::min(s.tank_masses_kg.size(), d.tank_mass_dots_kgps.size());
    for (std::size_t i = 0; i < n; ++i) {
        out.tank_masses_kg[i] = s.tank_masses_kg[i] + d.tank_mass_dots_kgps[i] * scale;
    }
    for (std::size_t i = n; i < s.tank_masses_kg.size(); ++i) {
        out.tank_masses_kg[i] = s.tank_masses_kg[i];
    }
    return out;
}

// Computes y_new = y + h * sum(coef_i * k_i) for a 7-stage RK step. Stages
// with coef == 0 are skipped.
ExtendedState combine_stages(
    const ExtendedState& y,
    double h,
    const ExtendedDerivative* ks,
    const double* coefs,
    std::size_t n_stages)
{
    ExtendedState out = y;
    for (std::size_t s = 0; s < n_stages; ++s) {
        if (coefs[s] == 0.0) {
            continue;
        }
        const double scale = h * coefs[s];
        out.motion.position_m = out.motion.position_m + ks[s].motion_dot.d_position_mps * scale;
        out.motion.velocity_mps = out.motion.velocity_mps + ks[s].motion_dot.d_velocity_mps2 * scale;
        const std::size_t n = std::min(out.tank_masses_kg.size(), ks[s].tank_mass_dots_kgps.size());
        for (std::size_t i = 0; i < n; ++i) {
            out.tank_masses_kg[i] += ks[s].tank_mass_dots_kgps[i] * scale;
        }
    }
    return out;
}

ExtendedState zero_state_like(const ExtendedState& ref)
{
    ExtendedState out;
    out.tank_masses_kg.assign(ref.tank_masses_kg.size(), 0.0);
    return out;
}

ExtendedState state_delta(const ExtendedState& a, const ExtendedState& b)
{
    ExtendedState out;
    out.motion.position_m = a.motion.position_m - b.motion.position_m;
    out.motion.velocity_mps = a.motion.velocity_mps - b.motion.velocity_mps;
    out.tank_masses_kg.assign(b.tank_masses_kg.size(), 0.0);
    for (std::size_t i = 0; i < out.tank_masses_kg.size(); ++i) {
        const double av = i < a.tank_masses_kg.size() ? a.tank_masses_kg[i] : 0.0;
        out.tank_masses_kg[i] = av - b.tank_masses_kg[i];
    }
    return out;
}

void add_scaled_state_in_place(ExtendedState* out, const ExtendedState& delta, double scale)
{
    out->motion.position_m = out->motion.position_m + delta.motion.position_m * scale;
    out->motion.velocity_mps = out->motion.velocity_mps + delta.motion.velocity_mps * scale;
    const std::size_t n = std::min(out->tank_masses_kg.size(), delta.tank_masses_kg.size());
    for (std::size_t i = 0; i < n; ++i) {
        out->tank_masses_kg[i] += delta.tank_masses_kg[i] * scale;
    }
}

void scale_state_in_place(ExtendedState* out, double scale)
{
    out->motion.position_m = out->motion.position_m * scale;
    out->motion.velocity_mps = out->motion.velocity_mps * scale;
    for (double& mass : out->tank_masses_kg) {
        mass *= scale;
    }
}

ExtendedState derivative_delta(
    const ExtendedDerivative& derivative,
    double scale,
    const ExtendedState& ref)
{
    ExtendedState out = zero_state_like(ref);
    out.motion.position_m = derivative.motion_dot.d_position_mps * scale;
    out.motion.velocity_mps = derivative.motion_dot.d_velocity_mps2 * scale;
    const std::size_t n = std::min(out.tank_masses_kg.size(), derivative.tank_mass_dots_kgps.size());
    for (std::size_t i = 0; i < n; ++i) {
        out.tank_masses_kg[i] = derivative.tank_mass_dots_kgps[i] * scale;
    }
    return out;
}

ExtendedState weighted_derivative_delta(
    const std::array<ExtendedDerivative, kDop853StagesExtended>& ks,
    const std::array<double, kDop853StagesExtended>& coefs,
    std::size_t n_stages,
    double scale,
    const ExtendedState& ref)
{
    ExtendedState out = zero_state_like(ref);
    for (std::size_t s = 0; s < n_stages; ++s) {
        if (coefs[s] == 0.0) {
            continue;
        }
        add_scaled_state_in_place(&out, derivative_delta(ks[s], scale, ref), coefs[s]);
    }
    return out;
}

ExtendedState dopri5_dense_interpolate(
    const ExtendedState& y_n,
    double h,
    const std::array<ExtendedDerivative, 7>& ks,
    double theta)
{
    double powers[4];
    powers[0] = theta;
    for (int i = 1; i < 4; ++i) {
        powers[i] = powers[i - 1] * theta;
    }

    double coefs[7] = {};
    for (std::size_t stage = 0; stage < ks.size(); ++stage) {
        for (int j = 0; j < 4; ++j) {
            coefs[stage] += kDopri5DenseP[stage][j] * powers[j];
        }
    }
    return combine_stages(y_n, h, ks.data(), coefs, ks.size());
}

std::array<ExtendedState, kDop853InterpolatorPower> make_dop853_dense_coefficients(
    const ExtendedState& y_n,
    const ExtendedState& y_new,
    double h,
    const std::array<ExtendedDerivative, kDop853StagesExtended>& ks)
{
    std::array<ExtendedState, kDop853InterpolatorPower> f;
    const ExtendedState delta_y = state_delta(y_new, y_n);
    f[0] = delta_y;

    f[1] = derivative_delta(ks[0], h, y_n);
    add_scaled_state_in_place(&f[1], delta_y, -1.0);

    f[2] = delta_y;
    scale_state_in_place(&f[2], 2.0);
    add_scaled_state_in_place(&f[2], derivative_delta(ks[0], h, y_n), -1.0);
    add_scaled_state_in_place(&f[2], derivative_delta(ks[12], h, y_n), -1.0);

    for (std::size_t i = 0; i < kDop853D.size(); ++i) {
        f[3 + i] = weighted_derivative_delta(
            ks, kDop853D[i], kDop853StagesExtended, h, y_n);
    }
    return f;
}

ExtendedState dop853_dense_interpolate(
    const ExtendedState& y_n,
    const std::array<ExtendedState, kDop853InterpolatorPower>& f,
    double theta)
{
    ExtendedState y = zero_state_like(y_n);
    for (int rev = kDop853InterpolatorPower - 1, count = 0; rev >= 0; --rev, ++count) {
        add_scaled_state_in_place(&y, f[static_cast<std::size_t>(rev)], 1.0);
        scale_state_in_place(&y, (count % 2 == 0) ? theta : (1.0 - theta));
    }
    ExtendedState out = y_n;
    add_scaled_state_in_place(&out, y, 1.0);
    return out;
}

// WRMS error norm using per-group atol + global rtol.
double error_norm(
    const ExtendedState& y,
    const ExtendedState& y_new,
    const ExtendedDerivative& err_per_unit_h,
    double h,
    const IntegratorTolerances& tol)
{
    auto scale = [](double atol, double rtol, double a, double b) {
        return atol + rtol * std::max(std::abs(a), std::abs(b));
    };

    auto sq = [](double v) { return v * v; };

    double sum_sq = 0.0;
    std::size_t count = 0;

    // Position components
    {
        const double sx = scale(tol.atol_position_m, tol.rtol, y.motion.position_m.x, y_new.motion.position_m.x);
        const double sy = scale(tol.atol_position_m, tol.rtol, y.motion.position_m.y, y_new.motion.position_m.y);
        const double sz = scale(tol.atol_position_m, tol.rtol, y.motion.position_m.z, y_new.motion.position_m.z);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_position_mps.x / sx);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_position_mps.y / sy);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_position_mps.z / sz);
        count += 3;
    }
    // Velocity components
    {
        const double sx = scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.x, y_new.motion.velocity_mps.x);
        const double sy = scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.y, y_new.motion.velocity_mps.y);
        const double sz = scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.z, y_new.motion.velocity_mps.z);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_velocity_mps2.x / sx);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_velocity_mps2.y / sy);
        sum_sq += sq(h * err_per_unit_h.motion_dot.d_velocity_mps2.z / sz);
        count += 3;
    }
    // Tank masses
    for (std::size_t i = 0; i < y.tank_masses_kg.size(); ++i) {
        const double y0 = y.tank_masses_kg[i];
        const double y1 = i < y_new.tank_masses_kg.size() ? y_new.tank_masses_kg[i] : y0;
        const double sc = scale(tol.atol_tank_mass_kg, tol.rtol, y0, y1);
        const double err_i = i < err_per_unit_h.tank_mass_dots_kgps.size()
            ? err_per_unit_h.tank_mass_dots_kgps[i] : 0.0;
        sum_sq += sq(h * err_i / sc);
        count += 1;
    }

    if (count == 0) {
        return 0.0;
    }
    return std::sqrt(sum_sq / static_cast<double>(count));
}

double dop853_error_norm(
    const ExtendedState& y,
    const ExtendedState& y_new,
    const std::array<ExtendedDerivative, kDop853StagesExtended>& ks,
    double h,
    const IntegratorTolerances& tol)
{
    auto scale = [](double atol, double rtol, double a, double b) {
        return atol + rtol * std::max(std::abs(a), std::abs(b));
    };
    auto accum_vec = [&ks](const std::array<double, kDop853Stages + 1>& coefs, bool velocity) {
        post2::vehicle::Vec3 out;
        for (std::size_t i = 0; i < coefs.size(); ++i) {
            const post2::vehicle::Vec3 v = velocity
                ? ks[i].motion_dot.d_velocity_mps2
                : ks[i].motion_dot.d_position_mps;
            out = out + v * coefs[i];
        }
        return out;
    };
    auto add_component = [](double e5, double e3, double sc, double* e5_sq, double* e3_sq) {
        const double a = e5 / sc;
        const double b = e3 / sc;
        *e5_sq += a * a;
        *e3_sq += b * b;
    };

    double err5_sq = 0.0;
    double err3_sq = 0.0;
    std::size_t count = 0;

    const post2::vehicle::Vec3 pos5 = accum_vec(kDop853E5, false);
    const post2::vehicle::Vec3 pos3 = accum_vec(kDop853E3, false);
    add_component(pos5.x, pos3.x,
        scale(tol.atol_position_m, tol.rtol, y.motion.position_m.x, y_new.motion.position_m.x),
        &err5_sq, &err3_sq);
    add_component(pos5.y, pos3.y,
        scale(tol.atol_position_m, tol.rtol, y.motion.position_m.y, y_new.motion.position_m.y),
        &err5_sq, &err3_sq);
    add_component(pos5.z, pos3.z,
        scale(tol.atol_position_m, tol.rtol, y.motion.position_m.z, y_new.motion.position_m.z),
        &err5_sq, &err3_sq);
    count += 3;

    const post2::vehicle::Vec3 vel5 = accum_vec(kDop853E5, true);
    const post2::vehicle::Vec3 vel3 = accum_vec(kDop853E3, true);
    add_component(vel5.x, vel3.x,
        scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.x, y_new.motion.velocity_mps.x),
        &err5_sq, &err3_sq);
    add_component(vel5.y, vel3.y,
        scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.y, y_new.motion.velocity_mps.y),
        &err5_sq, &err3_sq);
    add_component(vel5.z, vel3.z,
        scale(tol.atol_velocity_mps, tol.rtol, y.motion.velocity_mps.z, y_new.motion.velocity_mps.z),
        &err5_sq, &err3_sq);
    count += 3;

    for (std::size_t i = 0; i < y.tank_masses_kg.size(); ++i) {
        double mass5 = 0.0;
        double mass3 = 0.0;
        for (std::size_t stage = 0; stage < kDop853E5.size(); ++stage) {
            const double dot = i < ks[stage].tank_mass_dots_kgps.size()
                ? ks[stage].tank_mass_dots_kgps[i]
                : 0.0;
            mass5 += dot * kDop853E5[stage];
            mass3 += dot * kDop853E3[stage];
        }
        const double y1 = i < y_new.tank_masses_kg.size() ? y_new.tank_masses_kg[i] : y.tank_masses_kg[i];
        add_component(mass5, mass3,
            scale(tol.atol_tank_mass_kg, tol.rtol, y.tank_masses_kg[i], y1),
            &err5_sq, &err3_sq);
        ++count;
    }

    if (count == 0 || (err5_sq == 0.0 && err3_sq == 0.0)) {
        return 0.0;
    }
    const double denom = err5_sq + 0.01 * err3_sq;
    return std::abs(h) * err5_sq / std::sqrt(denom * static_cast<double>(count));
}

// Brent-Dekker root-finder for g(theta) on theta in [0, 1].
// Converges to |theta_hi - theta_lo| < 1e-12 (i.e. |Δt| < 1e-12 * h_used).
double brent_zero(
    double g_lo,
    double g_hi,
    const std::function<double(double)>& g_at_theta)
{
    constexpr double kThetaTol = 1.0e-12;
    constexpr double kFunctionTol = 1.0e-14;

    if (g_lo == 0.0) {
        return 0.0;
    }
    if (g_hi == 0.0) {
        return 1.0;
    }

    double a = 0.0;
    double b = 1.0;
    double fa = g_lo;
    double fb = g_hi;
    if (std::abs(fa) < std::abs(fb)) {
        std::swap(a, b);
        std::swap(fa, fb);
    }

    double c = a;
    double fc = fa;
    double d = c;
    bool mflag = true;

    for (int iter = 0; iter < 80; ++iter) {
        double s = b;
        if (fa != fc && fb != fc) {
            s =
                a * fb * fc / ((fa - fb) * (fa - fc)) +
                b * fa * fc / ((fb - fa) * (fb - fc)) +
                c * fa * fb / ((fc - fa) * (fc - fb));
        } else if (fb != fa) {
            s = b - fb * (b - a) / (fb - fa);
        }

        const double guard = (3.0 * a + b) * 0.25;
        const double lower = std::min(guard, b);
        const double upper = std::max(guard, b);
        const bool outside_guard = !(s > lower && s < upper);
        const bool too_slow_mflag = mflag && std::abs(s - b) >= 0.5 * std::abs(b - c);
        const bool too_slow = !mflag && std::abs(s - b) >= 0.5 * std::abs(c - d);
        const bool bracket_tiny_mflag = mflag && std::abs(b - c) < kThetaTol;
        const bool bracket_tiny = !mflag && std::abs(c - d) < kThetaTol;
        if (!std::isfinite(s) || outside_guard || too_slow_mflag || too_slow ||
            bracket_tiny_mflag || bracket_tiny) {
            s = 0.5 * (a + b);
            mflag = true;
        } else {
            mflag = false;
        }

        const double fs = g_at_theta(s);
        d = c;
        c = b;
        fc = fb;
        if ((fa < 0.0 && fs > 0.0) || (fa > 0.0 && fs < 0.0)) {
            b = s;
            fb = fs;
        } else {
            a = s;
            fa = fs;
        }

        if (std::abs(fa) < std::abs(fb)) {
            std::swap(a, b);
            std::swap(fa, fb);
        }
        if (std::abs(fb) < kFunctionTol || std::abs(b - a) < kThetaTol) {
            return std::clamp(b, 0.0, 1.0);
        }
    }
    return std::clamp(b, 0.0, 1.0);
}

// Common helper: examine events for sign changes between t_n and t_n+h_used,
// pick the earliest crossing (Brent on dense output), and return the
// corresponding EventHit. Returns nullopt if no event fires.
std::optional<EventHit> find_first_event(
    const std::vector<EventFunction>& events,
    double t_n,
    double h_used,
    const ExtendedState& y_n,
    const ExtendedState& y_new,
    const std::function<ExtendedState(double theta)>& interp)
{
    std::optional<EventHit> best;
    for (std::size_t i = 0; i < events.size(); ++i) {
        const auto& ev = events[i];
        if (!ev.g || !ev.terminating) {
            continue;
        }
        const double g0 = ev.g(t_n, y_n);
        const double g1 = ev.g(t_n + h_used, y_new);
        if (!std::isfinite(g0) || !std::isfinite(g1)) {
            continue;
        }
        const bool rising = (g0 < 0.0 && g1 >= 0.0) || (g0 <= 0.0 && g1 > 0.0);
        const bool falling = (g0 > 0.0 && g1 <= 0.0) || (g0 >= 0.0 && g1 < 0.0);
        const bool sign_change =
            ev.direction > 0 ? rising :
            ev.direction < 0 ? falling :
            (rising || falling);
        if (!sign_change) {
            continue;
        }
        const double theta = brent_zero(g0, g1, [&](double th) {
            const ExtendedState s = interp(th);
            return ev.g(t_n + th * h_used, s);
        });
        EventHit hit;
        hit.event_index = i;
        hit.name = ev.name;
        hit.t_s = t_n + theta * h_used;
        hit.state = interp(theta);
        if (!best.has_value() || hit.t_s < best->t_s) {
            best = hit;
        }
    }
    return best;
}

} // namespace

// =========================================================================
// Dopri5Integrator
// =========================================================================

Dopri5Integrator::Dopri5Integrator(IntegratorTolerances tolerances)
    : tolerances_(tolerances)
{
}

StepResult Dopri5Integrator::step(
    const ExtendedState& state,
    double t_s,
    double h_suggested,
    const DynamicsFunction& dynamics,
    const std::vector<EventFunction>& events)
{
    double h = std::abs(h_suggested);
    if (h <= 0.0) {
        StepResult res;
        res.state_end = state;
        res.t_end = t_s;
        res.h_used = 0.0;
        res.h_next_suggested = 0.0;
        res.accepted = true;
        return res;
    }

    // Cap retries; in practice 10 is generous.
    for (int attempt = 0; attempt < 12; ++attempt) {
        const ExtendedDerivative k1 = dynamics(t_s, state);

        const ExtendedState y2 = add_state_scaled_derivative(state, k1, h * kA21);
        const ExtendedDerivative k2 = dynamics(t_s + kC2 * h, y2);

        const double a31[2] = {kA31, kA32};
        const ExtendedDerivative kt31[2] = {k1, k2};
        const ExtendedState y3 = combine_stages(state, h, kt31, a31, 2);
        const ExtendedDerivative k3 = dynamics(t_s + kC3 * h, y3);

        const double a41[3] = {kA41, kA42, kA43};
        const ExtendedDerivative kt41[3] = {k1, k2, k3};
        const ExtendedState y4 = combine_stages(state, h, kt41, a41, 3);
        const ExtendedDerivative k4 = dynamics(t_s + kC4 * h, y4);

        const double a51[4] = {kA51, kA52, kA53, kA54};
        const ExtendedDerivative kt51[4] = {k1, k2, k3, k4};
        const ExtendedState y5 = combine_stages(state, h, kt51, a51, 4);
        const ExtendedDerivative k5 = dynamics(t_s + kC5 * h, y5);

        const double a61[5] = {kA61, kA62, kA63, kA64, kA65};
        const ExtendedDerivative kt61[5] = {k1, k2, k3, k4, k5};
        const ExtendedState y6 = combine_stages(state, h, kt61, a61, 5);
        const ExtendedDerivative k6 = dynamics(t_s + kC6 * h, y6);

        const double b_solution[6] = {kB1, kA72, kB3, kB4, kB5, kB6};
        const ExtendedDerivative kt_sol[6] = {k1, k2, k3, k4, k5, k6};
        ExtendedState y_new = combine_stages(state, h, kt_sol, b_solution, 6);

        // FSAL stage k7 = f at the end-of-step state (used by both the error
        // estimate and dense output).
        const ExtendedDerivative k7 = dynamics(t_s + h, y_new);
        const std::array<ExtendedDerivative, 7> ks{{k1, k2, k3, k4, k5, k6, k7}};

        // Error per unit step: e_i * k_i summed.
        ExtendedDerivative err;
        err.motion_dot.d_position_mps =
            k1.motion_dot.d_position_mps * kE1 +
            k3.motion_dot.d_position_mps * kE3 +
            k4.motion_dot.d_position_mps * kE4 +
            k5.motion_dot.d_position_mps * kE5 +
            k6.motion_dot.d_position_mps * kE6 +
            k7.motion_dot.d_position_mps * kE7;
        err.motion_dot.d_velocity_mps2 =
            k1.motion_dot.d_velocity_mps2 * kE1 +
            k3.motion_dot.d_velocity_mps2 * kE3 +
            k4.motion_dot.d_velocity_mps2 * kE4 +
            k5.motion_dot.d_velocity_mps2 * kE5 +
            k6.motion_dot.d_velocity_mps2 * kE6 +
            k7.motion_dot.d_velocity_mps2 * kE7;
        err.tank_mass_dots_kgps.assign(state.tank_masses_kg.size(), 0.0);
        for (std::size_t i = 0; i < err.tank_mass_dots_kgps.size(); ++i) {
            auto val = [&](const ExtendedDerivative& d) {
                return i < d.tank_mass_dots_kgps.size() ? d.tank_mass_dots_kgps[i] : 0.0;
            };
            err.tank_mass_dots_kgps[i] =
                val(k1) * kE1 + val(k3) * kE3 + val(k4) * kE4 +
                val(k5) * kE5 + val(k6) * kE6 + val(k7) * kE7;
        }

        const double e_norm = error_norm(state, y_new, err, h, tolerances_);

        const double accept_threshold = 1.0;
        if (e_norm <= accept_threshold) {
            // Suggest next step. If e_norm == 0, multiplicative factor would
            // be infinite — clamp to facmax.
            const double factor = e_norm > 0.0
                ? std::min(kFacMax, std::max(kFacMin, kSafetyFactor * std::pow(1.0 / e_norm, kOrderInv)))
                : kFacMax;
            const double h_next = h * factor;

            // Event detection on the accepted step via the Dormand-Prince
            // quartic dense-output polynomial.
            auto interp = [&](double theta) {
                (void)y_new;
                return dopri5_dense_interpolate(state, h, ks, theta);
            };
            const auto event = find_first_event(events, t_s, h, state, y_new, interp);

            StepResult res;
            if (event.has_value()) {
                res.state_end = event->state;
                res.t_end = event->t_s;
                res.h_used = event->t_s - t_s;
                res.event = event;
            } else {
                res.state_end = y_new;
                res.t_end = t_s + h;
                res.h_used = h;
            }
            res.h_next_suggested = h_next;
            res.accepted = true;
            return res;
        }

        // Step rejected: shrink and retry.
        const double factor = std::max(kFacMin, kSafetyFactor * std::pow(1.0 / e_norm, kOrderInv));
        const double h_new = h * factor;
        // Guard against zero-progress
        if (h_new < 1.0e-15) {
            StepResult res;
            res.state_end = state;
            res.t_end = t_s;
            res.h_used = 0.0;
            res.h_next_suggested = h;
            res.accepted = false;
            return res;
        }
        h = h_new;
    }

    // Exhausted retries: emit the last attempted state as a "best effort"
    // accepted step. This avoids hanging the integrator on hyper-stiff
    // problems we never expect in our domain.
    StepResult res;
    res.state_end = state;
    res.t_end = t_s;
    res.h_used = 0.0;
    res.h_next_suggested = h;
    res.accepted = false;
    return res;
}

// =========================================================================
// Rk4IntegratorAdapter — fixed-step RK4 with linear event detection
// =========================================================================

// =========================================================================
// Dop853Integrator
// =========================================================================

Dop853Integrator::Dop853Integrator(IntegratorTolerances tolerances)
    : tolerances_(tolerances)
{
}

StepResult Dop853Integrator::step(
    const ExtendedState& state,
    double t_s,
    double h_suggested,
    const DynamicsFunction& dynamics,
    const std::vector<EventFunction>& events)
{
    double h = std::abs(h_suggested);
    if (h <= 0.0) {
        StepResult res;
        res.state_end = state;
        res.t_end = t_s;
        res.h_used = 0.0;
        res.h_next_suggested = 0.0;
        res.accepted = true;
        return res;
    }

    for (int attempt = 0; attempt < 12; ++attempt) {
        std::array<ExtendedDerivative, kDop853StagesExtended> ks;
        ks[0] = dynamics(t_s, state);
        for (int s = 1; s < kDop853Stages; ++s) {
            const ExtendedState y_stage = combine_stages(
                state,
                h,
                ks.data(),
                kDop853A[static_cast<std::size_t>(s)].data(),
                static_cast<std::size_t>(s));
            ks[static_cast<std::size_t>(s)] =
                dynamics(t_s + kDop853C[static_cast<std::size_t>(s)] * h, y_stage);
        }

        ExtendedState y_new =
            combine_stages(state, h, ks.data(), kDop853B.data(), kDop853B.size());
        ks[12] = dynamics(t_s + h, y_new);

        const double e_norm = dop853_error_norm(state, y_new, ks, h, tolerances_);
        if (e_norm <= 1.0) {
            const double factor = e_norm > 0.0
                ? std::min(kFacMax, std::max(kFacMin, kSafetyFactor * std::pow(1.0 / e_norm, kDop853OrderInv)))
                : kFacMax;
            const double h_next = h * factor;

            for (int s = kDop853Stages + 1; s < kDop853StagesExtended; ++s) {
                const ExtendedState y_stage = combine_stages(
                    state,
                    h,
                    ks.data(),
                    kDop853A[static_cast<std::size_t>(s)].data(),
                    static_cast<std::size_t>(s));
                ks[static_cast<std::size_t>(s)] =
                    dynamics(t_s + kDop853C[static_cast<std::size_t>(s)] * h, y_stage);
            }
            const auto dense = make_dop853_dense_coefficients(state, y_new, h, ks);
            auto interp = [&](double theta) {
                return dop853_dense_interpolate(state, dense, theta);
            };
            const auto event = find_first_event(events, t_s, h, state, y_new, interp);

            StepResult res;
            if (event.has_value()) {
                res.state_end = event->state;
                res.t_end = event->t_s;
                res.h_used = event->t_s - t_s;
                res.event = event;
            } else {
                res.state_end = y_new;
                res.t_end = t_s + h;
                res.h_used = h;
            }
            res.h_next_suggested = h_next;
            res.accepted = true;
            return res;
        }

        const double factor = std::max(kFacMin, kSafetyFactor * std::pow(1.0 / e_norm, kDop853OrderInv));
        const double h_new = h * factor;
        if (h_new < 1.0e-15) {
            StepResult res;
            res.state_end = state;
            res.t_end = t_s;
            res.h_used = 0.0;
            res.h_next_suggested = h;
            res.accepted = false;
            return res;
        }
        h = h_new;
    }

    StepResult res;
    res.state_end = state;
    res.t_end = t_s;
    res.h_used = 0.0;
    res.h_next_suggested = h;
    res.accepted = false;
    return res;
}

StepResult Rk4IntegratorAdapter::step(
    const ExtendedState& state,
    double t_s,
    double h_suggested,
    const DynamicsFunction& dynamics,
    const std::vector<EventFunction>& events)
{
    const double h = h_suggested;
    Rk4OdeIntegrator integrator(OdeIntegratorOptions{h});
    const ExtendedState y_new = integrator.step(state, t_s, h, dynamics);

    auto interp = [&](double theta) {
        // Linear interpolation between (state, y_new) - rough but consistent
        // with RK4 being a coarse 4th-order solver: position/velocity scale
        // doesn't justify a fancier interpolant in event detection here.
        ExtendedState out;
        out.motion.position_m =
            state.motion.position_m * (1.0 - theta) + y_new.motion.position_m * theta;
        out.motion.velocity_mps =
            state.motion.velocity_mps * (1.0 - theta) + y_new.motion.velocity_mps * theta;
        out.tank_masses_kg.resize(state.tank_masses_kg.size());
        for (std::size_t i = 0; i < out.tank_masses_kg.size(); ++i) {
            const double y0 = state.tank_masses_kg[i];
            const double y1 = i < y_new.tank_masses_kg.size() ? y_new.tank_masses_kg[i] : y0;
            out.tank_masses_kg[i] = y0 * (1.0 - theta) + y1 * theta;
        }
        return out;
    };
    const auto event = find_first_event(events, t_s, h, state, y_new, interp);

    StepResult res;
    if (event.has_value()) {
        res.state_end = event->state;
        res.t_end = event->t_s;
        res.h_used = event->t_s - t_s;
        res.event = event;
    } else {
        res.state_end = y_new;
        res.t_end = t_s + h;
        res.h_used = h;
    }
    res.h_next_suggested = h;
    res.accepted = true;
    return res;
}

// =========================================================================
// Factory
// =========================================================================

std::unique_ptr<IIntegrator> make_integrator(
    const std::string& type,
    double /*step_s*/,
    const IntegratorTolerances& tolerances)
{
    if (type == "dopri5") {
        return std::make_unique<Dopri5Integrator>(tolerances);
    }
    if (type == "dop853") {
        return std::make_unique<Dop853Integrator>(tolerances);
    }
    if (type == "rk4" || type == "ode") {
        return std::make_unique<Rk4IntegratorAdapter>();
    }
    return std::make_unique<Rk4IntegratorAdapter>();
}

} // namespace post2::integrators
