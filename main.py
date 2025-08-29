project_name = "Simulink EV charger project"
import os, textwrap, json, pathlib

base = "/mnt/data/ev_charger_simulink"
os.makedirs(base, exist_ok=True)

files = {}

files["init_ev_charger.m"] = r"""
%% init_ev_charger.m
% Initialization script for "EV_Charger_Multifunctional" Simulink model

%% Simulation & solver
Ts_ctrl   = 1e-4;     % Control sampling time (100 us)
Ts_pwm    = 1e-5;     % PWM carrier step (10 us)
Ts_power  = 1e-5;     % Power stage discrete step
f_pwm     = 20e3;     % 20 kHz PWM

%% Grid parameters (50 Hz system)
V_grid_ll_rms   = 400;      % V line-line RMS
V_grid_ph_rms   = V_grid_ll_rms/sqrt(3);
V_grid_ph_peak  = sqrt(2)*V_grid_ph_rms;
f_grid          = 50;       % Hz
w_grid          = 2*pi*f_grid;

%% DC-link
Vdc_nom     = 700;     % V (enough headroom for 400Vac LL)
C_dc        = 2200e-6; % F
R_dc_damp   = 10;      % Ohm (bleeder/damper)

%% PV array (example: 10 strings x 12 modules per string)
% Replace with your datasheet values
Np_str   = 10;    % parallel strings
Ns_mod   = 12;    % series modules per string
Vmp_mod  = 31.1;  % V at MPP
Imp_mod  = 8.2;   % A at MPP
Voc_mod  = 38.5;  % V open circuit
Isc_mod  = 8.7;   % A short circuit

Vmp_array = Ns_mod*Vmp_mod;             % V
Imp_array = Np_str*Imp_mod;             % A
Pmp_array = Vmp_array*Imp_array;        % W

%% Battery pack (example Li-ion 96s x 3p)
N_series  = 96;
N_parallel= 3;
V_cell_nom= 3.7;      % V
C_cell_Ah = 50;       % Ah per parallel branch
V_batt_nom= N_series*V_cell_nom;
C_batt_Ah = N_parallel*C_cell_Ah;
C_batt_As = C_batt_Ah*3600; % As

SOC_init  = 0.6;      % 60% initial SOC
I_batt_max_chg = 120; % A
I_batt_max_dis = 120; % A

eta_chg   = 0.97;
eta_dis   = 0.97;

%% DC-DC converters
% PV boost
L_pv   = 2.2e-3;   % H
R_Lpv  = 0.05;     % Ohm
C_pv   = 470e-6;   % F (input side small cap)

% Bi-directional buck-boost for battery <-> DC bus
L_bb   = 1.5e-3;   % H
R_Lbb  = 0.03;     % Ohm
C_bb   = 220e-6;   % F

%% Inverter (3-phase)
L_f    = 2.5e-3;   % H per phase (L filter)
R_f    = 0.2;      % Ohm
% If using LCL, extend params accordingly

%% Controllers
% MPPT
Vpv_ref_init = Vmp_array;  % start near MPP
mppt_step    = 1.0;        % volts per step

% DC-link voltage controller (outer loop) to set inverter d-axis current
Vdc_ref      = Vdc_nom;
Kp_vdc       = 0.5;
Ki_vdc       = 50;

% Inverter current control (inner dq PI)
Kp_id        = 5;
Ki_id        = 800;
Kp_iq        = 5;
Ki_iq        = 800;

% Battery charge control
Vbatt_ref    = V_batt_nom*1.05;  % CV level (approx)
Ibatt_ref_cc = 0.5*I_batt_max_chg; % CC level

%% Protection thresholds
Vdc_max  = 800;
Vdc_min  = 500;
SOC_min  = 0.2;
SOC_max  = 0.9;
Igrid_max= 30;    % A (phase)

%% Mode manager thresholds
P_solar_on    = 0.10*Pmp_array;   % W to consider "PV available"
P_load_ev     = 7e3;              % W nominal EV load example

disp('Initialization complete. Load this script before running the model.');
"""

files["mppt_pno_step.m"] = r"""
function [Vref, state] = mppt_pno_step(V, I, state)
%MPPT_PNO_STEP One step of Perturb & Observe MPPT
%  Inputs:
%    V, I  - PV voltage and current (instantaneous or filtered)
%    state - struct with fields: Vprev, Iprev, Pprev, Vref, step
%  Outputs:
%    Vref  - updated voltage reference for PV boost converter
%    state - updated state
%
%  Use inside a MATLAB Function block. Initialize 'state' in Simulink using
%  a Data Store Memory/InitFcn or from the workspace.

if isempty(state)
    state.Vprev = V;
    state.Iprev = I;
    state.Pprev = V*I;
    state.Vref  = V;
    state.step  = 1.0; % volts
end

P = V*I;
dP = P - state.Pprev;
dV = V - state.Vprev;

if dP == 0
    % keep previous direction
elseif dP > 0
    % If we moved in a good direction, keep perturbing same way
    if dV > 0
        state.Vref = state.Vref + state.step;
    else
        state.Vref = state.Vref - state.step;
    end
else % dP < 0
    % Reverse direction
    if dV > 0
        state.Vref = state.Vref - state.step;
    else
        state.Vref = state.Vref + state.step;
    end
end

% Optional: clamp Vref between safe limits (e.g., 0.7*Vmp..0.98*Voc*Nseries)
Vref = state.Vref;

state.Vprev = V;
state.Iprev = I;
state.Pprev = P;
"""

files["bms_soc_update.m"] = r"""
function soc = bms_soc_update(soc_prev, Ibatt, dt, C_as, eta_chg, eta_dis)
%BMS_SOC_UPDATE Coulomb counting SOC update
%  soc_prev in [0..1], Ibatt (A), dt (s), C_as (As), efficiencies
%  Positive current means charging the battery.

if Ibatt >= 0
    delta = (Ibatt*dt*eta_chg)/C_as;
else
    delta = (Ibatt*dt/eta_dis)/C_as;
end

soc = soc_prev + delta;

% clamp
if soc > 1, soc = 1; end
if soc < 0, soc = 0; end
"""

files["dq_current_pi.m"] = r"""
function [vd_ref, vq_ref, x] = dq_current_pi(id_ref, iq_ref, id_meas, iq_meas, w, L, R, x, Ts, Kp_id, Ki_id, Kp_iq, Ki_iq)
%DQ_CURRENT_PI Decoupled PI current control in dq frame for a 3ph inverter
%  x contains integrator states: x.id_int, x.iq_int
%  w is electrical angular frequency (rad/s)
%  L, R are filter parameters per phase
%  Ts is control step

if isempty(x)
    x.id_int = 0;
    x.iq_int = 0;
end

% Errors
ed = id_ref - id_meas;
eq = iq_ref - iq_meas;

% PI
vd_pi = Kp_id*ed + x.id_int;
vq_pi = Kp_iq*eq + x.iq_int;

% Decoupling/feedforward terms
vd_ff = R*id_meas - w*L*iq_meas;
vq_ff = R*iq_meas + w*L*id_meas;

vd_ref = vd_pi + vd_ff;
vq_ref = vq_pi + vq_ff;

% Update integrators (Tustin or simple Euler)
x.id_int = x.id_int + Ki_id*ed*Ts;
x.iq_int = x.iq_int + Ki_iq*eq*Ts;
"""

files["vdc_pi_outer.m"] = r"""
function [id_ref, x] = vdc_pi_outer(Vdc_ref, Vdc_meas, x, Ts, Kp, Ki, Imax)
%VDC_PI_OUTER Outer voltage loop sets d-axis current reference
%  x.integrator
if isempty(x)
    x.integrator = 0;
end

e = Vdc_ref - Vdc_meas;
u = Kp*e + x.integrator;
x.integrator = x.integrator + Ki*e*Ts;

% Limit
id_ref = max(min(u, Imax), -Imax);
"""

files["mode_manager_logic.m"] = r"""
function [mode, grid_enable, pv_enable, batt_chg_cmd, batt_dis_cmd] = mode_manager_logic(Ppv, Pload, SOC, grid_ok, Vdc, Vdc_min, Vdc_max, SOC_min, SOC_max)
%MODE_MANAGER_LOGIC Simple rule-based mode selection
%  Returns mode:
%   1 = GRID_TIED
%   2 = OFFGRID_PV
%   3 = OFFGRID_PV_BATT
%   4 = CHARGE_BATT_FROM_GRID
%   5 = IDLE/PROTECT

mode = 5;
grid_enable = false; pv_enable = false;
batt_chg_cmd = false; batt_dis_cmd = false;

if Vdc > Vdc_max || Vdc < 0.5*Vdc_min
    mode = 5; % protect
    return;
end

if grid_ok
    if Ppv >= 0.1*Pload
        % Grid-tied with PV priority; export/import as needed
        mode = 1; grid_enable = true; pv_enable = true;
        if SOC < SOC_max && Ppv > Pload
            batt_chg_cmd = true;
        end
    else
        % Not enough PV; either charge battery or supply from grid
        mode = 4; grid_enable = true;
        if SOC < SOC_max
            batt_chg_cmd = true;
        end
    end
else
    % Islanded
    if Ppv >= Pload
        mode = 2; pv_enable = true;
        if SOC < SOC_max
            batt_chg_cmd = true;
        end
    else
        mode = 3; pv_enable = true;
        if SOC > SOC_min
            batt_dis_cmd = true;
        else
            mode = 5; % can't supply load
        end
    end
end
"""

files["README.txt"] = textwrap.dedent(r"""
    EV Charger (Solar PV Array Based Multifunctional) — Simulink Build Guide
    =========================================================================

    This folder contains MATLAB code you can use INSIDE Simulink to assemble a
    multifunctional EV charger with:
      • PV boost converter + MPPT
      • Bidirectional battery DC-DC (buck/boost)
      • 3-phase inverter for grid-tied or islanded operation
      • Mode manager (grid-tied / off-grid / battery charge-discharge)
      • DC-link regulation

    Files
    -----
    1) init_ev_charger.m      – Initialize all parameters (run first).
    2) mppt_pno_step.m        – MATLAB Function for Perturb & Observe MPPT.
    3) bms_soc_update.m       – MATLAB Function for battery SOC (Coulomb counting).
    4) dq_current_pi.m        – MATLAB Function for dq current PI control (inner loop).
    5) vdc_pi_outer.m         – MATLAB Function for DC-link voltage PI (outer loop).
    6) mode_manager_logic.m   – MATLAB Function with rule-based mode selection.
    7) README.txt             – This guide.

    How to Use
    ----------
    1) Open MATLAB.
    2) Add this folder to your path, e.g.:
         addpath('<<path-to-this-folder>>')
    3) Run:
         init_ev_charger
    4) Create a new Simulink model: EV_Charger_Multifunctional.slx
    5) In the model, set:
         • powergui: Discrete, sample time = Ts_power from workspace.
         • Solver: Discrete (fixed-step), fixed-step size = Ts_power.
    6) Assemble subsystems as below.

    Top-Level Blocks
    ----------------
    [PV Subsystem] --> [PV Boost DC-DC] ----> [DC Link Cdc] ----> [3φ Inverter] ---> Grid / EV AC port
                                   |                             |
                           [Bidirectional DC-DC] <---- Battery --+
                                   |
                                [Battery]

    1) PV Subsystem:
       - Use 'PV Array' block (Simscape Electrical / Specialized Power Systems / Renewable Energy).
         Set array to Ns_mod and Np_str from workspace (see init script).
       - Measure Vpv, Ipv (current sensor & voltage sensor). Low-pass filter (e.g., 200 Hz cutoff).
       - MATLAB Function block "MPPT_PNO":
             [Vref, st] = mppt_pno_step(Vpv_f, Ipv_f, st);
         Feed Vref to the PV boost duty controller (e.g., voltage-mode control on PV input).

    2) PV Boost Converter:
       - Components: Inductor L_pv (Series R R_Lpv), diode/MOSFET (Universal Bridge used as chopper),
         input cap C_pv, output to DC-link (C_dc).
       - Use "PWM Generator (DC-DC)" or implement duty with "Compare To Constant" vs carrier.
       - Control: Track Vpv -> Vref from MPPT by adjusting duty 'd'. A simple PI on (Vref - Vpv).

    3) DC Link:
       - Capacitor C_dc with damping resistor R_dc_damp. Measure Vdc.

    4) Bidirectional DC-DC (Battery):
       - Use half-bridge (two MOSFETs) + inductor L_bb + capacitor C_bb.
       - Control logic:
           • If batt_chg_cmd: regulate battery current to +Ibatt_ref_cc (CC) and then CV to Vbatt_ref.
           • If batt_dis_cmd: regulate DC-link to Vdc_ref using battery power (boost mode).
       - Update SOC:
           soc = bms_soc_update(soc_prev, Ibatt, Ts_ctrl, C_batt_As, eta_chg, eta_dis);

    5) Inverter (3-phase to grid / AC port):
       - Use "Universal Bridge" (IGBTs) with DC bus = Vdc, L filter (L_f, R_f) to grid.
       - Synchronization: Use PLL block (Simscape / Control) or generate theta = 2π50t for test.
       - Control:
           Outer loop: [id_ref] = vdc_pi_outer(Vdc_ref, Vdc, x_vdc, Ts_ctrl, Kp_vdc, Ki_vdc, Igrid_max)
           Inner loop: [vd_ref, vq_ref, x_i] = dq_current_pi(id_ref, iq_ref=0, id, iq, w_grid, L_f, R_f, x_i, Ts_ctrl, ...)
           Convert vdq -> vabc, then generate SPWM with "PWM Generator (3-phase, 2-level)".

    6) Mode Manager:
       - MATLAB Function block "mode_manager_logic" with inputs:
           Ppv, Pload, SOC, grid_ok, Vdc, thresholds
         Outputs:
           mode, grid_enable, pv_enable, batt_chg_cmd, batt_dis_cmd
       - Use outputs to enable/disable subsystems and select references.

    7) Protections and Limits:
       - Disable gating if Vdc > Vdc_max or < Vdc_min.
       - Current limits on grid (Igrid_max) and battery (I_batt_max_*).
       - Soft-start for Vdc controller.

    Suggested Logging
    -----------------
    • Scopes: Vpv, Ipv, Ppv, Vdc; id, iq, vd, vq; SOC; Ibatt; grid currents; mode.
    • Use "To Workspace" blocks for post-processing.

    Notes
    -----
    • Parameter values in init_ev_charger.m are examples; tune using your component data.
    • If you're new to PLL, you can start with a fixed grid angle (theta = 2*pi*50*t) to validate controllers,
      then replace with a proper PLL block.
    • Use rate transition blocks between Ts_power (plant) and Ts_ctrl (control) where needed.

    """)

# Write files
for name, content in files.items():
    with open(os.path.join(base, name), "w", encoding="utf-8") as f:
        f.write(content)

sorted(os.listdir(base)), base
