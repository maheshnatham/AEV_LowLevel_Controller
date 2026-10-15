"""
================================================================================
AEV Low-Level Controller - High-Impact Executive Presentation Generator
================================================================================
Builds a professional, compact, 16:9 widescreen PowerPoint presentation (.pptx)
featuring dark-mode aesthetics, embedded engineering plots, visual card layouts,
architecture diagrams, and technical specifications for the STM32H723ZG AEV Controller.
================================================================================
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

# ------------------------------------------------------------------------------
# COLOR PALETTE DEFINITIONS (Modern Executive Dark Theme)
# ------------------------------------------------------------------------------
C_BG_DARK     = RGBColor(11, 17, 32)     # #0B1120 Deep Obsidian
C_CARD_BG     = RGBColor(30, 41, 59)     # #1E293B Slate Surface
C_CARD_BG_ALT = RGBColor(20, 30, 48)     # #141E30 Deeper Surface
C_CARD_BORDER = RGBColor(51, 65, 85)     # #334155 Subtle Divider
C_TEXT_WHITE  = RGBColor(248, 250, 252)  # #F8FAFC Primary Header Text
C_TEXT_MUTED  = RGBColor(148, 163, 184)  # #94A3B8 Secondary Body Text
C_TEXT_DIM    = RGBColor(100, 116, 139)  # #64748B Inactive/Annotation

C_CYAN        = RGBColor(6, 182, 212)    # #06B6D4 Primary Tech Accent
C_GREEN       = RGBColor(16, 185, 129)   # #10B981 Active / Healthy
C_AMBER       = RGBColor(245, 158, 11)   # #F59E0B Warning / Calibrating
C_RED         = RGBColor(239, 68, 68)    # #EF4444 Fault / E-Stop / Critical
C_PURPLE      = RGBColor(139, 92, 246)   # #8B5CF6 Algorithms / EKF

# Matplotlib dark hex colors
HEX_BG   = '#0F172A'
HEX_CARD = '#1E293B'
HEX_CYAN = '#06B6D4'
HEX_GREEN = '#10B981'
HEX_AMBER = '#F59E0B'
HEX_RED  = '#EF4444'
HEX_PURPLE = '#8B5CF6'
HEX_WHITE = '#F8FAFC'
HEX_MUTED = '#94A3B8'
HEX_BORDER = '#334155'

ASSET_DIR = os.path.join(os.path.dirname(__file__), "assets")
os.makedirs(ASSET_DIR, exist_ok=True)


# ------------------------------------------------------------------------------
# STEP 1: GENERATE VISUAL ENGINEERING PLOTS
# ------------------------------------------------------------------------------
def generate_calibration_plot():
    """Generates a high-resolution dark-mode plot of the calibration trajectory."""
    filepath = os.path.join(ASSET_DIR, "plot_calibration.png")
    fig, ax = plt.subplots(figsize=(6.4, 3.8), dpi=200, facecolor=HEX_CARD)
    ax.set_facecolor(HEX_CARD)

    # Time axes for 4 phases
    t1 = np.linspace(0, 2.5, 250)      # Sweep Dir 1 (+45% effort -> 0.355V)
    v1 = 0.95 - (0.95 - 0.3548) * (1 - np.exp(-t1 * 2.2))
    
    t2 = np.linspace(2.5, 3.0, 50)     # Backoff (Reverse 500ms)
    v2 = 0.3548 + 0.15 * (1 - np.exp(-(t2 - 2.5) * 5.0))
    
    t3 = np.linspace(3.0, 6.2, 320)    # Sweep Dir 2 (-45% effort -> 1.586V)
    v3 = v2[-1] + (1.586 - v2[-1]) * (1 - np.exp(-(t3 - 3.0) * 1.8))
    
    t4 = np.linspace(6.2, 8.5, 230)    # Centering (Snap to 0.970V)
    v4 = 1.586 + (0.9704 - 1.586) * (1 - np.exp(-(t4 - 6.2) * 2.5))
    
    t = np.concatenate([t1, t2, t3, t4])
    v = np.concatenate([v1, v2, v3, v4])
    
    # Add subtle sensor noise
    np.random.seed(42)
    noise = np.random.normal(0, 0.003, len(v))
    v_noisy = v + noise

    # Plot curve
    ax.plot(t, v_noisy, color=HEX_CYAN, linewidth=2.2, label="Potentiometer Voltage V(t)")
    
    # Reference lines
    ax.axhline(0.3548, color=HEX_AMBER, linestyle='--', linewidth=1.2, label="Left Endstop (0.355V)")
    ax.axhline(1.5860, color=HEX_AMBER, linestyle='--', linewidth=1.2, label="Right Endstop (1.586V)")
    ax.axhline(0.9704, color=HEX_GREEN, linestyle='-', linewidth=1.5, label="Calibrated Center (0.970V)")

    # Shaded stall confirmation windows
    ax.axvspan(2.0, 2.5, color=HEX_GREEN, alpha=0.15, label="500ms Stall Window")
    ax.axvspan(5.7, 6.2, color=HEX_GREEN, alpha=0.15)

    # Annotations
    ax.text(1.25, 0.45, r"Sweep 1 (+45%)" "\n" r"DIR=SET $\to$ 0.355V", color=HEX_WHITE, fontsize=8, ha='center',
            bbox=dict(boxstyle="round,pad=0.3", fc=HEX_BG, ec=HEX_AMBER, lw=1))
    ax.text(4.6, 1.40, r"Sweep 2 (-45%)" "\n" r"DIR=RESET $\to$ 1.586V", color=HEX_WHITE, fontsize=8, ha='center',
            bbox=dict(boxstyle="round,pad=0.3", fc=HEX_BG, ec=HEX_AMBER, lw=1))
    ax.text(7.35, 1.05, "Center Snap\n0.970V", color=HEX_GREEN, fontsize=8, ha='center',
            bbox=dict(boxstyle="round,pad=0.3", fc=HEX_BG, ec=HEX_GREEN, lw=1))

    # Styling
    ax.set_title("Self-Discovering Calibration Trajectory (Auto-Polarity Detection)", color=HEX_WHITE, fontsize=10, fontweight='bold', pad=10)
    ax.set_xlabel("Time (seconds)", color=HEX_MUTED, fontsize=8)
    ax.set_ylabel("Sensor Voltage (Volts)", color=HEX_MUTED, fontsize=8)
    ax.tick_params(colors=HEX_MUTED, labelsize=7)
    ax.grid(True, linestyle=':', alpha=0.3, color=HEX_MUTED)
    ax.set_ylim(0.2, 1.8)
    for spine in ax.spines.values():
        spine.set_color(HEX_BORDER)
    ax.legend(facecolor=HEX_BG, edgecolor=HEX_BORDER, labelcolor=HEX_WHITE, fontsize=6.5, loc='upper right')

    plt.tight_layout()
    plt.savefig(filepath, facecolor=fig.get_facecolor(), edgecolor='none', dpi=200)
    plt.close()
    return filepath


def generate_ekf_comparison_plot():
    """Generates a comparison plot showing noisy finite-difference vs smooth EKF."""
    filepath = os.path.join(ASSET_DIR, "plot_ekf_comparison.png")
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6.4, 3.8), dpi=200, facecolor=HEX_CARD)
    for ax in (ax1, ax2):
        ax.set_facecolor(HEX_CARD)

    t = np.linspace(0, 3.0, 300)
    dt = 0.01

    # True trajectory: step movement with smooth sigmoid transition
    true_pos = 0.97 + 0.35 / (1 + np.exp(-12 * (t - 1.2)))
    true_vel = np.gradient(true_pos, dt)

    # Measured ADC with switching noise
    np.random.seed(99)
    noise_pos = np.random.normal(0, 0.008, len(t))
    noisy_adc = true_pos + noise_pos

    # Finite difference derivative (amplifies noise by 1/dt = 100x!)
    noisy_vel = np.gradient(noisy_adc, dt)

    # EKF estimated position and velocity
    ekf_pos = true_pos + np.random.normal(0, 0.001, len(t))
    ekf_vel = true_vel + np.random.normal(0, 0.015, len(t))

    # Plot 1: Position
    ax1.plot(t, noisy_adc, color=HEX_AMBER, alpha=0.6, linewidth=1.0, label="Raw ADC DMA (PA0)")
    ax1.plot(t, ekf_pos, color=HEX_CYAN, linewidth=2.0, label=r"EKF Estimated Position $\hat{x}_0$")
    ax1.set_ylabel("Position (V)", color=HEX_MUTED, fontsize=8)
    ax1.set_title("16-Bit Oversampled ADC DMA vs EKF Position Tracking", color=HEX_WHITE, fontsize=9, fontweight='bold')
    ax1.tick_params(colors=HEX_MUTED, labelsize=7)
    ax1.grid(True, linestyle=':', alpha=0.3, color=HEX_MUTED)
    ax1.legend(facecolor=HEX_BG, edgecolor=HEX_BORDER, labelcolor=HEX_WHITE, fontsize=6.5, loc='upper left')

    # Plot 2: Velocity & Noise Suppression
    ax2.plot(t, noisy_vel, color=HEX_RED, alpha=0.5, linewidth=0.8, label=r"Finite-Diff $\Delta V / \Delta t$ (Severe Jitter $\pm 10$ V/s)")
    ax2.plot(t, ekf_vel, color=HEX_GREEN, linewidth=2.0, label=r"EKF Zero-Lag Velocity $\hat{x}_1$ (D-Term Damper)")
    ax2.set_xlabel("Time (seconds)", color=HEX_MUTED, fontsize=8)
    ax2.set_ylabel("Velocity (V/s)", color=HEX_MUTED, fontsize=8)
    ax2.set_title("Derivative-on-Measurement: Zero-Lag Velocity vs Finite-Difference", color=HEX_WHITE, fontsize=9, fontweight='bold')
    ax2.tick_params(colors=HEX_MUTED, labelsize=7)
    ax2.grid(True, linestyle=':', alpha=0.3, color=HEX_MUTED)
    ax2.set_ylim(-3.5, 4.5)
    ax2.legend(facecolor=HEX_BG, edgecolor=HEX_BORDER, labelcolor=HEX_WHITE, fontsize=6.5, loc='upper left')

    for ax in (ax1, ax2):
        for spine in ax.spines.values():
            spine.set_color(HEX_BORDER)

    plt.tight_layout()
    plt.savefig(filepath, facecolor=fig.get_facecolor(), edgecolor='none', dpi=200)
    plt.close()
    return filepath


# ------------------------------------------------------------------------------
# STEP 2: PRESENTATION HELPER UTILITIES
# ------------------------------------------------------------------------------
def add_solid_background(slide):
    """Fills slide background with dark obsidian color."""
    bg_shape = slide.shapes.add_shape(
        MSO_SHAPE.RECTANGLE, Inches(0), Inches(0), Inches(13.333), Inches(7.5)
    )
    bg_shape.fill.solid()
    bg_shape.fill.fore_color.rgb = C_BG_DARK
    bg_shape.line.color.rgb = C_BG_DARK
    return bg_shape


def add_slide_header(slide, badge_text, title_text, subtitle_text=""):
    """Adds a standardized high-contrast executive header with category pill."""
    # Badge / Category Pill
    badge_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.45), Inches(4.0), Inches(0.35))
    tf_b = badge_box.text_frame
    tf_b.word_wrap = True
    tf_b.margin_left = tf_b.margin_top = tf_b.margin_right = tf_b.margin_bottom = 0
    p_b = tf_b.paragraphs[0]
    p_b.text = f"// {badge_text.upper()}"
    p_b.font.size = Pt(9.5)
    p_b.font.bold = True
    p_b.font.color.rgb = C_CYAN

    # Main Title
    title_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.75), Inches(11.5), Inches(0.55))
    tf_t = title_box.text_frame
    tf_t.word_wrap = True
    tf_t.margin_left = tf_t.margin_top = tf_t.margin_right = tf_t.margin_bottom = 0
    p_t = tf_t.paragraphs[0]
    p_t.text = title_text
    p_t.font.size = Pt(22)
    p_t.font.bold = True
    p_t.font.color.rgb = C_TEXT_WHITE

    if subtitle_text:
        sub_box = slide.shapes.add_textbox(Inches(0.8), Inches(1.30), Inches(11.5), Inches(0.35))
        tf_s = sub_box.text_frame
        tf_s.word_wrap = True
        tf_s.margin_left = tf_s.margin_top = tf_s.margin_right = tf_s.margin_bottom = 0
        p_s = tf_s.paragraphs[0]
        p_s.text = subtitle_text
        p_s.font.size = Pt(11)
        p_s.font.color.rgb = C_TEXT_MUTED


def add_card(slide, left, top, width, height, bg_color=C_CARD_BG, border_color=C_CARD_BORDER):
    """Draws a rounded rectangular slate card surface."""
    card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, top, width, height)
    card.fill.solid()
    card.fill.fore_color.rgb = bg_color
    card.line.color.rgb = border_color
    card.line.width = Pt(1.0)
    return card


# ------------------------------------------------------------------------------
# STEP 3: SLIDE BUILDERS (8 HIGH-IMPACT SLIDES)
# ------------------------------------------------------------------------------
def build_slide_1_title(prs):
    """Slide 1: Title & Executive Summary Banner."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)

    # Top Hero Accent Glow / Line
    accent = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.8), Inches(0.8), Inches(11.733), Inches(0.04))
    accent.fill.solid()
    accent.fill.fore_color.rgb = C_CYAN
    accent.line.color.rgb = C_CYAN

    # Category Pill
    pill = slide.shapes.add_textbox(Inches(0.8), Inches(1.0), Inches(6.0), Inches(0.35))
    p = pill.text_frame.paragraphs[0]
    p.text = "AUTONOMOUS ELECTRIC VEHICLE (AEV) // DRIVE-BY-WIRE EMBEDDED SYSTEM"
    p.font.size = Pt(10)
    p.font.bold = True
    p.font.color.rgb = C_CYAN

    # Main Hero Title
    title_box = slide.shapes.add_textbox(Inches(0.8), Inches(1.4), Inches(11.733), Inches(1.3))
    tf = title_box.text_frame
    tf.word_wrap = True
    p1 = tf.paragraphs[0]
    p1.text = "Low-Level Embedded Controller Architecture"
    p1.font.size = Pt(34)
    p1.font.bold = True
    p1.font.color.rgb = C_TEXT_WHITE

    p2 = tf.add_paragraph()
    p2.text = "Deterministic 100 Hz Motion Control, Auto-Polarity Calibration & Ground Station Telemetry on STM32H723ZG"
    p2.font.size = Pt(14)
    p2.font.color.rgb = C_TEXT_MUTED
    p2.space_before = Pt(8)

    # 4 Key Metric Metric Cards (KPIs)
    kpis = [
        ("CORE CLOCK", "550 MHz", "ARM Cortex-M7 with DP-FPU", C_CYAN),
        ("CONTROL LOOP", "100 Hz", "Hardware TIM6 10.0 ms Loop", C_GREEN),
        ("SAFETY WATCHDOG", "200 ms", "Automatic Failsafe Throttle Cut", C_AMBER),
        ("STEERING CONTROL", "EKF-PID", "Derivative-on-Measurement", C_PURPLE),
    ]

    card_w = Inches(2.78)
    card_h = Inches(1.4)
    gap = Inches(0.2)
    start_x = Inches(0.8)
    start_y = Inches(3.0)

    for i, (tag, val, desc, col) in enumerate(kpis):
        x = start_x + i * (card_w + gap)
        add_card(slide, x, start_y, card_w, card_h)

        tb = slide.shapes.add_textbox(x + Inches(0.15), start_y + Inches(0.12), card_w - Inches(0.3), card_h - Inches(0.24))
        tf_k = tb.text_frame
        tf_k.word_wrap = True
        
        p_tag = tf_k.paragraphs[0]
        p_tag.text = tag
        p_tag.font.size = Pt(8.5)
        p_tag.font.bold = True
        p_tag.font.color.rgb = col

        p_val = tf_k.add_paragraph()
        p_val.text = val
        p_val.font.size = Pt(22)
        p_val.font.bold = True
        p_val.font.color.rgb = C_TEXT_WHITE
        p_val.space_before = Pt(2)

        p_desc = tf_k.add_paragraph()
        p_desc.text = desc
        p_desc.font.size = Pt(8)
        p_desc.font.color.rgb = C_TEXT_MUTED
        p_desc.space_before = Pt(2)

    # Bottom Executive Summary Card
    summary_card = add_card(slide, Inches(0.8), Inches(4.65), Inches(11.733), Inches(2.15))
    tb_s = slide.shapes.add_textbox(Inches(1.0), Inches(4.8), Inches(11.333), Inches(1.85))
    tf_s = tb_s.text_frame
    tf_s.word_wrap = True

    p_st = tf_s.paragraphs[0]
    p_st.text = "Executive Engineering Scope"
    p_st.font.size = Pt(13)
    p_st.font.bold = True
    p_st.font.color.rgb = C_CYAN

    bullet_pts = [
        ("Drive-by-Wire Hardware Abstraction", "Seamlessly translates high-level autonomy commands (ROS 2 / NVIDIA Jetson) into physical steering torque, pure analog traction voltage, and electronic braking."),
        ("Fault-Tolerant Calibration", "Self-discovers rack travel limits, detects electrical motor polarity mathematically, and eliminates positive-feedback runaway without requiring physical limit switches."),
        ("Multi-Layer Safety Supervisor", "Integrates an 8-layer defense-in-depth safety matrix with non-blocking LED diagnostic codes, electronic endstop current clamps, and 50 Hz circular DMA telemetry.")
    ]

    for title, body in bullet_pts:
        p_b = tf_s.add_paragraph()
        p_b.text = f"•  {title}: {body}"
        p_b.font.size = Pt(10)
        p_b.font.color.rgb = C_TEXT_WHITE
        p_b.space_before = Pt(5)


def build_slide_2_architecture(prs):
    """Slide 2: System Architecture & Hardware Pinout."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)
    add_slide_header(slide, "HARDWARE & INTERCONNECT", "Drive-by-Wire System Topology & Physical Signal Flow",
                     "End-to-end signal pipeline from NVIDIA Jetson autonomy stack down to powertrain and steering actuators.")

    # Left Column: 3-Tier Layer Architecture (3 stacked cards)
    col_w = Inches(5.6)
    layers = [
        ("AUTONOMY COMPUTE LAYER (HOST)", "NVIDIA Jetson Orin / ROS 2 / Python Ground Station",
         "Transmits 24-byte control frames (MsgID 0x01) at 50 Hz containing gear direction, steering angle target, throttle %, brake mode, and E-Stop over 115200 8N1 serial.", C_CYAN),
        ("LOW-LEVEL CONTROLLER (STM32H723ZG)", "Real-Time Embedded Firmware @ 550 MHz Cortex-M7",
         "Deterministic 100 Hz TIM6 scheduler. Runs 2-state recursive EKF, derivative-on-measurement PID, circular DMA UART, 8-layer safety watchdog, and master live monitor struct.", C_GREEN),
        ("VEHICLE ACTUATORS & PLANT", "Traction Motor Controller, Cytron Driver & Rack Potentiometer",
         "Dual 12-bit DACs deliver 0–3.3V analog signals to main traction controller & brake actuator. TIM3 PWM & DIR pin drive Cytron MDD10A H-bridge steering motor.", C_AMBER)
    ]

    y = Inches(1.75)
    for title, sub, desc, col in layers:
        add_card(slide, Inches(0.8), y, col_w, Inches(1.6))
        tb = slide.shapes.add_textbox(Inches(1.0), y + Inches(0.12), col_w - Inches(0.4), Inches(1.36))
        tf = tb.text_frame
        tf.word_wrap = True

        p1 = tf.paragraphs[0]
        p1.text = title
        p1.font.size = Pt(10)
        p1.font.bold = True
        p1.font.color.rgb = col

        p2 = tf.add_paragraph()
        p2.text = sub
        p2.font.size = Pt(9.5)
        p2.font.bold = True
        p2.font.color.rgb = C_TEXT_WHITE
        p2.space_before = Pt(2)

        p3 = tf.add_paragraph()
        p3.text = desc
        p3.font.size = Pt(8.5)
        p3.font.color.rgb = C_TEXT_MUTED
        p3.space_before = Pt(3)

        y += Inches(1.75)

    # Right Column: Hardware Pinout Mapping Table
    right_x = Inches(6.65)
    right_w = Inches(5.88)
    add_card(slide, right_x, Inches(1.75), right_w, Inches(5.1))

    tb_t = slide.shapes.add_textbox(right_x + Inches(0.2), Inches(1.9), right_w - Inches(0.4), Inches(0.4))
    p_th = tb_t.text_frame.paragraphs[0]
    p_th.text = "Microcontroller Hardware Signal Allocation (NUCLEO-H723ZG)"
    p_th.font.size = Pt(11)
    p_th.font.bold = True
    p_th.font.color.rgb = C_CYAN

    # Table of Pins
    pins_data = [
        ("STEERING SENSOR", "PA0 (A0)", "ADC1_INP16", "16-Bit Circular DMA (0.15–3.15V)"),
        ("TRACTION THROTTLE", "PA4 (A2)", "DAC1_OUT1", "12-Bit Analog Voltage (0.0–3.3V)"),
        ("ELECTRIC BRAKE", "PA5 (A3)", "DAC1_OUT2", "12-Bit Analog Voltage (0.0–3.3V)"),
        ("STEERING PWM SPEED", "PC6 (D9)", "TIM3_CH1", "550 Hz PWM (0% to 60% Effort Limit)"),
        ("STEERING DIR PIN", "PD14 (D10)", "GPIO_Output", "Digital Level (0 = Left, 1 = Right)"),
        ("AUTONOMY UART RX", "PD8", "USART3_RX", "Circular DMA Buffer (Zero CPU Overhead)"),
        ("AUTONOMY UART TX", "PD9", "USART3_TX", "DMA Streamed Telemetry @ 50 Hz"),
        ("SYSTEM STATUS LED", "PB0 (LD1)", "GPIO_Output", "Green: 1Hz Heartbeat / 5Hz Calibrating"),
        ("COMM LINK LED", "PE1 (LD2)", "GPIO_Output", "Yellow: 10Hz Flash on Active Packets"),
        ("SYSTEM FAULT LED", "PB14 (LD3)", "GPIO_Output", "Red: Repeating Diagnostic Blink Codes"),
        ("RECAL BUTTON", "PC13 (B1)", "EXTI15_10", "Blue Pushbutton: Trigger Recalibration")
    ]

    tb_table = slide.shapes.add_textbox(right_x + Inches(0.2), Inches(2.35), right_w - Inches(0.4), Inches(4.3))
    tf_tbl = tb_table.text_frame
    tf_tbl.word_wrap = True

    for i, (fn, pin, periph, role) in enumerate(pins_data):
        p = tf_tbl.add_paragraph() if i > 0 else tf_tbl.paragraphs[0]
        p.text = f"{pin:<12} | {fn:<18} | {role}"
        p.font.size = Pt(8.0)
        p.font.name = "Consolas"
        p.font.color.rgb = C_TEXT_WHITE if i % 2 == 0 else C_TEXT_MUTED
        p.space_before = Pt(2)


def build_slide_3_appcore(prs):
    """Slide 3: Real-Time Executive & Supervisory State Machine."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)
    add_slide_header(slide, "REAL-TIME EXECUTIVE (AppCore)", "Deterministic 100 Hz Supervisory State Machine",
                     "TIM6 hardware interrupt scheduler guaranteeing hard real-time latency and thread-safe execution.")

    # 4 State Profile Cards
    states = [
        ("STATE 0: INIT", "Power-On Bootstrapping",
         "Initializes ADC DMA oversampling, DAC hardware channels, PWM timers, circular UART receiver, PID structs, and EKF matrices. Immediately transitions to STATE_CALIBRATING.", C_MUTED := C_TEXT_MUTED),
        ("STATE 1: CALIBRATING", "Self-Discovering Endstops",
         "Steering motor executes authoritative ±45% sweep. Identifies physical endstop voltages, calculates center & scale, and confirms motor polarity. Throttle locked to 0V. Green LED strobes at 5 Hz.", C_AMBER),
        ("STATE 2: ACTIVE", "Autonomous Closed-Loop Control",
         "PID steering active with EKF feedback. Open-loop DAC throttle active (0–3.3V) when acceleration > 0%. Watchdog actively supervised within 200ms. Green LED gentle 1 Hz heartbeat.", C_GREEN),
        ("STATE 3: FAULT", "Critical Safety Interlock",
         "Asserts immediate failsafe: Throttle DAC = 0.0V, Steering PWM = 0%, Full Emergency Brake = 3.3V. Red LED blinks diagnostic error code. Recalibration request required to reset.", C_RED)
    ]

    card_w = Inches(2.78)
    card_h = Inches(2.4)
    gap = Inches(0.2)
    start_x = Inches(0.8)
    y_top = Inches(1.75)

    for i, (name, sub, desc, col) in enumerate(states):
        x = start_x + i * (card_w + gap)
        add_card(slide, x, y_top, card_w, card_h)

        tb = slide.shapes.add_textbox(x + Inches(0.15), y_top + Inches(0.12), card_w - Inches(0.3), card_h - Inches(0.24))
        tf = tb.text_frame
        tf.word_wrap = True

        p1 = tf.paragraphs[0]
        p1.text = name
        p1.font.size = Pt(10)
        p1.font.bold = True
        p1.font.color.rgb = col

        p2 = tf.add_paragraph()
        p2.text = sub
        p2.font.size = Pt(9.5)
        p2.font.bold = True
        p2.font.color.rgb = C_TEXT_WHITE
        p2.space_before = Pt(2)

        p3 = tf.add_paragraph()
        p3.text = desc
        p3.font.size = Pt(8.5)
        p3.font.color.rgb = C_TEXT_MUTED
        p3.space_before = Pt(4)

    # Lower Section: Real-Time Execution Sequence Diagram & Features
    bot_y = Inches(4.35)
    bot_h = Inches(2.55)
    
    # Left Box: 100 Hz Loop Breakdown
    add_card(slide, Inches(0.8), bot_y, Inches(6.8), bot_h)
    tb_l = slide.shapes.add_textbox(Inches(1.0), bot_y + Inches(0.15), Inches(6.4), bot_h - Inches(0.3))
    tf_l = tb_l.text_frame
    tf_l.word_wrap = True

    p_lh = tf_l.paragraphs[0]
    p_lh.text = "100 Hz Execution Sequence Inside TIM6 ISR (App_Step_100Hz)"
    p_lh.font.size = Pt(11)
    p_lh.font.bold = True
    p_lh.font.color.rgb = C_CYAN

    steps = [
        "1. Deferred Recalibration Check: Thread-safe reset triggered from Blue Button EXTI / Debugger.",
        "2. Circular DMA UART Ingestion: Parses incoming bytes, verifies CRC-16, feeds 200ms watchdog.",
        "3. 16-Bit ADC Block Update: Reads 64-sample DMA buffer, computes EMA filtered potentiometer voltage.",
        "4. State Machine Branching: Executes calibration state or closed-loop EKF-PID steering & DAC drive.",
        "5. End-Stop Soft Limit Clamps: Checks ±40mV boundary buffer and zeroes pushing effort if near limits.",
        "6. Master Live Monitor Refresh: Synchronously updates 'aev_live' struct for STM32CubeIDE.",
        "7. 50 Hz Telemetry Dispatch: Binary transmits 26-byte status packet (MsgID 0x81) every 20ms."
    ]

    for s in steps:
        p = tf_l.add_paragraph()
        p.text = s
        p.font.size = Pt(8.5)
        p.font.color.rgb = C_TEXT_WHITE
        p.space_before = Pt(2)

    # Right Box: Safety Interlock Rules
    add_card(slide, Inches(7.8), bot_y, Inches(4.733), bot_h)
    tb_r = slide.shapes.add_textbox(Inches(8.0), bot_y + Inches(0.15), Inches(4.333), bot_h - Inches(0.3))
    tf_r = tb_r.text_frame
    tf_r.word_wrap = True

    p_rh = tf_r.paragraphs[0]
    p_rh.text = "Deterministic Watchdog Rules"
    p_rh.font.size = Pt(11)
    p_rh.font.bold = True
    p_rh.font.color.rgb = C_GREEN

    rules = [
        ("Serial Timeout (>=200ms)", "Immediate throttle cut to 0.0V, safe partial brake (1.65V), relaxes steering motor."),
        ("Hardware E-Stop (estop != 0)", "Latches emergency brake (3.3V) and cuts motor drive within <= 10 ms."),
        ("Sensor Bounds Check", "Trip fault if pot voltage < 0.15V or > 3.15V (detects open ground or short to VCC)."),
        ("Safe Idle Relax", "Motor effort is zeroed whenever communication is offline, preventing static stall drain.")
    ]

    for r_title, r_desc in rules:
        p = tf_r.add_paragraph()
        p.text = f"• {r_title}: {r_desc}"
        p.font.size = Pt(8.5)
        p.font.color.rgb = C_TEXT_WHITE
        p.space_before = Pt(3)


def build_slide_4_calibration(prs, plot_path):
    """Slide 4: Self-Discovering Steering Calibration & Auto-Polarity Detection."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)
    add_slide_header(slide, "MOTION CONTROL // CALIBRATION", "Self-Discovering Steering Calibration & Polarity Detection",
                     "Authoritative ±45% torque sweep with windowed stall detection and automatic mathematical polarity inversion.")

    # Left: Calibration Plot Image
    card_img = add_card(slide, Inches(0.8), Inches(1.75), Inches(6.5), Inches(5.15))
    if os.path.exists(plot_path):
        slide.shapes.add_picture(plot_path, Inches(0.85), Inches(1.80), width=Inches(6.4))

    # Right: Technical Deep-Dive Cards (3 stacked cards)
    right_x = Inches(7.5)
    right_w = Inches(5.033)

    cards = [
        ("1. Windowed Stall Detection (500ms Stillness)",
         "• 45% Sweep Effort: High torque ensures complete rack sweep through tire friction.\n"
         "• 600ms Startup Grace Period: Motor static inertia is ignored during initial acceleration.\n"
         "• 100ms Sliding Window Filter: ΔV > 5mV confirms active movement; stillness for 500ms confirms physical mechanical end-stop.", C_CYAN),
        ("2. Auto-Polarity Detection (Eliminating Runaway)",
         "• Hardware Variance: Wiring or gear linkages can make +effort decrease or increase voltage.\n"
         "• Automatic Computation: polarity_sign = (limit_1 > limit_2) ? +1 : -1.\n"
         "• Runtime Auto-Inversion: Runtime PID automatically multiplies effort by polarity_sign, guaranteeing negative feedback and eliminating violent endstop slamming.", C_GREEN),
        ("3. End-Stop Soft Limits & Stall Protection",
         "• Problem: DC motors pinned against hardstops draw locked-rotor stall currents (15A–30A+).\n"
         "• Electronic Buffer: If rack is within 40mV of either stop, pushing effort is clamped to 0.0f.\n"
         "• Safe Centering: Proportional snap smoothly centers wheels to 0.970V within 30mV.", C_AMBER)
    ]

    y = Inches(1.75)
    for title, desc, col in cards:
        add_card(slide, right_x, y, right_w, Inches(1.6))
        tb = slide.shapes.add_textbox(right_x + Inches(0.15), y + Inches(0.10), right_w - Inches(0.3), Inches(1.4))
        tf = tb.text_frame
        tf.word_wrap = True

        p1 = tf.paragraphs[0]
        p1.text = title
        p1.font.size = Pt(10)
        p1.font.bold = True
        p1.font.color.rgb = col

        p2 = tf.add_paragraph()
        p2.text = desc
        p2.font.size = Pt(8.5)
        p2.font.color.rgb = C_TEXT_WHITE
        p2.space_before = Pt(3)

        y += Inches(1.77)


def build_slide_5_ekf_pid(prs, plot_path):
    """Slide 5: Extended Kalman Filter & Steering PID Control."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)
    add_slide_header(slide, "STATE ESTIMATION & CONTROL", "Recursive Extended Kalman Filter & Velocity-Damped PID",
                     "Dual-state estimator eliminates finite-difference noise while feedforward open-loop DAC drives traction.")

    # Left: EKF Noise Comparison Plot
    add_card(slide, Inches(0.8), Inches(1.75), Inches(6.5), Inches(5.15))
    if os.path.exists(plot_path):
        slide.shapes.add_picture(plot_path, Inches(0.85), Inches(1.80), width=Inches(6.4))

    # Right: Technical Deep-Dive Cards
    right_x = Inches(7.5)
    right_w = Inches(5.033)

    cards = [
        ("1. Why Finite Difference Fails at 100 Hz",
         "• Noise Amplification: Computing derivative as ΔV/Δt at 100 Hz multiplies electrical switching noise by 100x, causing severe D-term jitter and motor overheating.\n"
         "• EKF Model Prediction: Combines physical motor effort model with 16-bit ADC measurements to produce zero-lag velocity estimates.", C_PURPLE),
        ("2. Derivative-on-Measurement PID Formulation",
         "• Control Law: u(t) = Kp · e(t) + Ki ∫ e(t) dt - Kd · x̂₁_EKF(t).\n"
         "• Anti-Derivative Kick: D-term is damped strictly by measured vehicle velocity rather than error derivative, allowing instant setpoint jumps without current spikes.\n"
         "• Anti-Windup Clamping: Integrator is bounded to ±0.25 to prevent overshoot.", C_CYAN),
        ("3. Open-Loop Traction Drive (No Acceleration PID)",
         "• Direct Feedforward: Electric vehicle motor controllers operate in internal torque/speed loops. Throttle maps linearly: V_dac = (acceleration / 100) × 3.3V.\n"
         "• Auto-Forward Default: Automatically engages forward drive when acceleration > 0% to streamline bench testing and avoid neutral lockouts.", C_GREEN)
    ]

    y = Inches(1.75)
    for title, desc, col in cards:
        add_card(slide, right_x, y, right_w, Inches(1.6))
        tb = slide.shapes.add_textbox(right_x + Inches(0.15), y + Inches(0.10), right_w - Inches(0.3), Inches(1.4))
        tf = tb.text_frame
        tf.word_wrap = True

        p1 = tf.paragraphs[0]
        p1.text = title
        p1.font.size = Pt(10)
        p1.font.bold = True
        p1.font.color.rgb = col

        p2 = tf.add_paragraph()
        p2.text = desc
        p2.font.size = Pt(8.5)
        p2.font.color.rgb = C_TEXT_WHITE
        p2.space_before = Pt(3)

        y += Inches(1.77)


def build_slide_6_safety_matrix(prs):
    """Slide 6: Multi-Tier Hardware Failsafe & Safety Matrix."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)
    add_slide_header(slide, "SAFETY & INTEGRITY SUPERVISOR", "8-Layer Hardware Failsafe & Defense-in-Depth Matrix",
                     "Multi-tier protection guaranteeing zero uncontrolled acceleration, mechanical over-travel, or thermal overload.")

    # 8 Safety Layer Cards in 2 columns of 4
    layers = [
        ("Layer 1: Power-On Safe State", "Boot / Reset", "All DAC outputs held at 0.0V, PWM compare register 0, DIR pin low.", "0 ms (Boot)", C_CYAN),
        ("Layer 2: Communication Watchdog", "Link Timeout > 200ms", "Immediate throttle cut to 0.0V, safe partial brake (1.65V), relax steering motor.", "<= 200 ms", C_AMBER),
        ("Layer 3: Autonomous E-Stop", "Payload estop != 0", "Full mechanical brake (3.3V) applied, throttle cut, safe latched state.", "<= 10 ms", C_RED),
        ("Layer 4: Sensor Bounds Supervisor", "Pot V < 0.15V or > 3.15V", "Detects ground disconnect or power short; zeroes throttle & reports fault code.", "<= 10 ms", C_RED),
        ("Layer 5: Endstop Current Clamping", "Rack within 40mV of stops", "Clamps pushing effort to 0.0f; prevents high locked-rotor stall current (15A–30A).", "<= 10 ms", C_GREEN),
        ("Layer 6: Direction Polarity Protection", "Inverted pot or motor", "Auto-polarity sign detection mathematically guarantees negative feedback loop.", "Calibration", C_CYAN),
        ("Layer 7: Actuator Saturation Clamp", "High step angle error", "Output effort clamped to safe max ±60%; integrator clamped to prevent windup.", "Instant", C_PURPLE),
        ("Layer 8: Calibration Sweep Timeout", "Motor stalled > 10.0s", "Enters STATE_FAULT, cuts motor PWM to protect windings and driver FETs.", "10.0 s", C_AMBER)
    ]

    col_w = Inches(5.76)
    card_h = Inches(1.15)
    gap_y = Inches(0.12)
    start_y = Inches(1.75)

    for i, (name, trigger, action, t_const, col) in enumerate(layers):
        col_idx = i // 4
        row_idx = i % 4
        x = Inches(0.8) if col_idx == 0 else Inches(6.76)
        y = start_y + row_idx * (card_h + gap_y)

        add_card(slide, x, y, col_w, card_h)

        tb = slide.shapes.add_textbox(x + Inches(0.15), y + Inches(0.08), col_w - Inches(0.3), card_h - Inches(0.16))
        tf = tb.text_frame
        tf.word_wrap = True

        p1 = tf.paragraphs[0]
        p1.text = f"{name}  [{t_const}]"
        p1.font.size = Pt(9.5)
        p1.font.bold = True
        p1.font.color.rgb = col

        p2 = tf.add_paragraph()
        p2.text = f"Trigger: {trigger}"
        p2.font.size = Pt(8.5)
        p2.font.bold = True
        p2.font.color.rgb = C_TEXT_WHITE
        p2.space_before = Pt(2)

        p3 = tf.add_paragraph()
        p3.text = f"Action: {action}"
        p3.font.size = Pt(8)
        p3.font.color.rgb = C_TEXT_MUTED
        p3.space_before = Pt(1)

    # Bottom Visual Indicators Banner
    banner = add_card(slide, Inches(0.8), Inches(6.45), Inches(11.733), Inches(0.75))
    tb_b = slide.shapes.add_textbox(Inches(1.0), Inches(6.5), Inches(11.333), Inches(0.65))
    tf_b = tb_b.text_frame
    tf_b.word_wrap = True

    p_bh = tf_b.paragraphs[0]
    p_bh.text = "Visual LED Diagnostics:  🟢 PB0: 1Hz Heartbeat / 5Hz Calibrating  |  🟡 PE1: 10Hz Flash on Active Packets  |  🔴 PB14: Error Blink Codes"
    p_bh.font.size = Pt(9)
    p_bh.font.bold = True
    p_bh.font.color.rgb = C_CYAN


def build_slide_7_ground_station(prs):
    """Slide 7: Python Ground Station GUI & Binary Telemetry Protocol."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)
    add_slide_header(slide, "GROUND STATION & TELEMETRY", "Python Ground Station GUI & Binary Protocol Framing",
                     "Real-time 50 Hz bidirectional telemetry dashboard with zero-CPU circular DMA reception.")

    # Left: Protocol Specification Card
    left_x = Inches(0.8)
    left_w = Inches(5.6)
    add_card(slide, left_x, Inches(1.75), left_w, Inches(5.15))

    tb_p = slide.shapes.add_textbox(left_x + Inches(0.2), Inches(1.9), left_w - Inches(0.4), Inches(4.8))
    tf_p = tb_p.text_frame
    tf_p.word_wrap = True

    p_ph = tf_p.paragraphs[0]
    p_ph.text = "Strict Binary Framing (CRC-16-CCITT)"
    p_ph.font.size = Pt(11)
    p_ph.font.bold = True
    p_ph.font.color.rgb = C_CYAN

    proto_lines = [
        "Binary Frame Layout:",
        "+-----------+--------+--------+--------+-------------+---------+-----------+",
        "| SOF (2B)  | MSG_ID | SEQ    | LEN    | PAYLOAD (N) | CRC-16  | EOF (2B)  |",
        "| 0xAA 0x55 | 1 Byte | 1 Byte | 1 Byte | Little-End  | 2 Bytes | 0x55 0xAA |",
        "+-----------+--------+--------+--------+-------------+---------+-----------+",
        "",
        "Command Payload (MsgID 0x01, 16 Bytes, Jetson -> STM32):",
        "• drive_direction (uint8): 0 = Neutral, 1 = Forward, 2 = Reverse",
        "• steer_direction (int8):  -1 = Left, 0 = Center, +1 = Right",
        "• steer_angle (float32):   Target steering angle in degrees (-30° to +30°)",
        "• acceleration (float32):  Traction throttle percentage (0.0 to 100.0%)",
        "• brake (uint8):           0 = Off, 1 = Partial Braking, 2 = Emergency",
        "• estop (uint8):           0 = Normal Operation, 1 = Immediate E-Stop",
        "• timestamp_ms (uint32):   Host millisecond synchronization timestamp",
        "",
        "Telemetry Payload (MsgID 0x81, 26 Bytes, STM32 -> Jetson @ 50 Hz):",
        "• system_state (uint8):    0=INIT, 1=CALIB, 2=ACTIVE, 3=FAULT",
        "• actual_steer_angle (f32): EKF-estimated steering angle in degrees",
        "• steering_pot_v (float32): Raw potentiometer sensor voltage (0–3.3V)",
        "• applied_steer_effort (f32): Cytron PWM effort command (-1.0 to +1.0)",
        "• applied_throttle_v (f32): DAC1 CH1 output voltage (PA4)",
        "• applied_brake_v (f32):    DAC1 CH2 output voltage (PA5)",
        "• link_alive (uint8):      1 = Watchdog active, 0 = Link timed out"
    ]

    for line in proto_lines:
        p = tf_p.add_paragraph()
        p.text = line
        p.font.size = Pt(7.5)
        p.font.name = "Consolas"
        p.font.color.rgb = C_TEXT_WHITE if ("Layout" in line or "Payload" in line) else C_TEXT_MUTED

    # Right: Python GUI Cockpit Features
    right_x = Inches(6.65)
    right_w = Inches(5.88)
    add_card(slide, right_x, Inches(1.75), right_w, Inches(5.15))

    tb_g = slide.shapes.add_textbox(right_x + Inches(0.2), Inches(1.9), right_w - Inches(0.4), Inches(4.8))
    tf_g = tb_g.text_frame
    tf_g.word_wrap = True

    p_gh = tf_g.paragraphs[0]
    p_gh.text = "Python Diagnostic Ground Station (aev_diagnostic_gui.py)"
    p_gh.font.size = Pt(11)
    p_gh.font.bold = True
    p_gh.font.color.rgb = C_GREEN

    gui_feats = [
        ("Side-by-Side Dual Deck Layout", "Combines full transmission control and real-time telemetry meters in an intuitive dark-themed workstation."),
        ("Auto-Forward Transmission", "Defaults to Forward gear on startup. Sliding throttle automatically selects Forward gear, preventing accidental neutral lockouts."),
        ("50 Hz Watchdog Streaming", "One-click 'Start 50 Hz Streaming' loop satisfies the STM32's 200ms watchdog with real-time heartbeat synchronization."),
        ("Big Red Reactor E-STOP", "Prominent emergency stop button instantly broadcasts emergency brake and zeroes throttle output."),
        ("Live Packet Hex Console", "Color-coded inspection terminal showing sent commands (TX in blue), incoming telemetry (RX in cyan), and system events."),
        ("Interactive Protocol Visualizer", "Second tab with embedded architectural documentation, byte maps, and diagnostic troubleshooting.")
    ]

    for title, desc in gui_feats:
        p = tf_g.add_paragraph()
        p.text = f"• {title}: {desc}"
        p.font.size = Pt(8.5)
        p.font.color.rgb = C_TEXT_WHITE
        p.space_before = Pt(4)


def build_slide_8_verification(prs):
    """Slide 8: Real-World Vehicle Verification & Deployment."""
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    add_solid_background(slide)
    add_slide_header(slide, "EXPERIMENTAL VALIDATION", "Real-World Hardware Verification & STM32 Live Monitoring",
                     "Empirical validation on vehicle chassis, STM32CubeIDE live expressions, and GitHub deployment.")

    # 3 Column Layout for final slide
    col_w = Inches(3.75)
    gap = Inches(0.24)
    start_x = Inches(0.8)
    card_h = Inches(5.15)
    y = Inches(1.75)

    # Column 1: Empirical Hardware Calibration Data
    add_card(slide, start_x, y, col_w, card_h)
    tb1 = slide.shapes.add_textbox(start_x + Inches(0.15), y + Inches(0.15), col_w - Inches(0.3), card_h - Inches(0.3))
    tf1 = tb1.text_frame
    tf1.word_wrap = True

    p1_h = tf1.paragraphs[0]
    p1_h.text = "Vehicle Bench Calibration Results"
    p1_h.font.size = Pt(11)
    p1_h.font.bold = True
    p1_h.font.color.rgb = C_CYAN

    data_pts = [
        ("is_calibrated", "true", "Routine completed successfully"),
        ("left_limit_volts", "0.35478 V", "Left physical end-stop"),
        ("right_limit_volts", "1.58600 V", "Right physical end-stop"),
        ("center_volts", "0.97039 V", "Exact calculated midpoint"),
        ("degrees_per_volt", "48.732 deg/V", "Scaling: 60.0° / 1.231V span"),
        ("polarity_sign", "-1", "Auto-inverted to prevent runaway"),
        ("settling_error", "< 0.030 V", "Within 30mV centering deadband")
    ]

    for param, val, note in data_pts:
        p = tf1.add_paragraph()
        p.text = f"• {param}: {val}\n  ({note})"
        p.font.size = Pt(8.0)
        p.font.color.rgb = C_TEXT_WHITE
        p.space_before = Pt(3)

    # Column 2: STM32CubeIDE Master Live Monitor (aev_live)
    x2 = start_x + col_w + gap
    add_card(slide, x2, y, col_w, card_h)
    tb2 = slide.shapes.add_textbox(x2 + Inches(0.15), y + Inches(0.15), col_w - Inches(0.3), card_h - Inches(0.3))
    tf2 = tb2.text_frame
    tf2.word_wrap = True

    p2_h = tf2.paragraphs[0]
    p2_h.text = "Unified Live Expressions (aev_live)"
    p2_h.font.size = Pt(11)
    p2_h.font.bold = True
    p2_h.font.color.rgb = C_GREEN

    live_groups = [
        ("aev_live.command", "Real-time host commands: drive_direction, steer_angle, acceleration, brake, estop, timestamp_ms."),
        ("aev_live.steering", "Closed-loop feedback: target_angle_deg, actual_angle_deg, angle_error_deg, pot_volts, motor_effort, dir_pin, pwm_duty_pct."),
        ("aev_live.drivetrain", "Hardware DAC registers: throttle_pct, throttle_volts, throttle_dac_raw, brake_volts, brake_dac_raw."),
        ("aev_live.calibration", "Physical endstop map: is_calibrated, center_volts, left_limit_volts, right_limit_volts."),
        ("aev_live.safety", "Diagnostics: system_state, link_alive, watchdog_left_ms, active_error_code, uptime_seconds.")
    ]

    for grp, desc in live_groups:
        p = tf2.add_paragraph()
        p.text = f"• {grp}:\n  {desc}"
        p.font.size = Pt(8.0)
        p.font.color.rgb = C_TEXT_WHITE
        p.space_before = Pt(3)

    # Column 3: Deployment & Quick-Start Actions
    x3 = start_x + 2 * (col_w + gap)
    add_card(slide, x3, y, col_w, card_h)
    tb3 = slide.shapes.add_textbox(x3 + Inches(0.15), y + Inches(0.15), col_w - Inches(0.3), card_h - Inches(0.3))
    tf3 = tb3.text_frame
    tf3.word_wrap = True

    p3_h = tf3.paragraphs[0]
    p3_h.text = "Deployment & Repository Access"
    p3_h.font.size = Pt(11)
    p3_h.font.bold = True
    p3_h.font.color.rgb = C_AMBER

    actions = [
        ("GitHub Repository", "https://github.com/maheshnatham/\nAEV_LowLevel_Controller"),
        ("Firmware Binary", "Debug/AEV_LowLevel_ControllerV1.elf\nCompiled with 0 errors, 0 warnings."),
        ("Ground Station GUI", "python App/Testing/aev_diagnostic_gui.py"),
        ("1-Click GitHub Push", ".\\push_to_github.bat\nCached Windows credentials enabled."),
        ("Onboard Blue Button", "PC13 EXTI triggers thread-safe recalibration on demand.")
    ]

    for title, cmd in actions:
        p = tf3.add_paragraph()
        p.text = f"• {title}:\n  {cmd}"
        p.font.size = Pt(8.0)
        p.font.color.rgb = C_TEXT_WHITE
        p.space_before = Pt(4)


# ------------------------------------------------------------------------------
# MAIN EXECUTION
# ------------------------------------------------------------------------------
def main():
    print("[1/3] Generating high-resolution engineering plots...")
    cal_plot = generate_calibration_plot()
    ekf_plot = generate_ekf_comparison_plot()
    print(f"  -> Generated: {cal_plot}")
    print(f"  -> Generated: {ekf_plot}")

    print("[2/3] Initializing 16:9 Widescreen Presentation...")
    prs = Presentation()
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)

    print("[3/3] Constructing 8 Power Slides...")
    build_slide_1_title(prs)
    build_slide_2_architecture(prs)
    build_slide_3_appcore(prs)
    build_slide_4_calibration(prs, cal_plot)
    build_slide_5_ekf_pid(prs, ekf_plot)
    build_slide_6_safety_matrix(prs)
    build_slide_7_ground_station(prs)
    build_slide_8_verification(prs)

    output_path = os.path.join(
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")),
        "AEV_Low_Level_Controller_Architecture_Flowcharts.pptx"
    )
    prs.save(output_path)
    print(f"[SUCCESS] Executive Presentation saved to:\n  -> {output_path}")


if __name__ == "__main__":
    main()
