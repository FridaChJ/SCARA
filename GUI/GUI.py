# =============================================================================
# FRONTEND
# =============================================================================
# Description : Tkinter-based GUI for controlling a robotic arm.
#               Sends commands via MQTT (stub) and displays motor angles
#               received from the backend.
# Authors     : Frida Sophia Chavez Juarez
# Menthor     : Oscar Vargas Perez
# Last updated: 4/6/2026 3:38
# Dependencies: tkinter (stdlib), Pillow
# =============================================================================

# =============================================================================
# IMPORTS AND SETUP
# =============================================================================

from tkinter import *  #Import all Tkinter GUI components
    #Builds GUI
from PIL import Image, ImageTk #Pillow for image loading and Tkinter compatibility
    #Handles images
from mqtt_client import MQTTClient #MQTT communication class
    #Sends and receives data
from controller import Controller #Controller class
    #Connects fontend with backend logic
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg #Embed plots in Tkinter
from matplotlib.figure import Figure #Create plot objects
import matplotlib
matplotlib.use("TkAgg")  # Tkinter backend
    #Connects matplotlib with Tkinter
    #Without it plots won't render in the GUI


# =============================================================================
# CONSTANTS
# =============================================================================
#Anythinhg that doesn't change at runtime is a constant 
# =============================================================================

WINDOW_SIZE     = "1920x1080" #Main window resolution
WINDOW_TITLE    = "Lo Siento Vic" #Window text 

# Background Colors
BG_MAIN         = "lightgray" #General UI background
BG_DISPLAY      = "white" #Plot/data display area
BG_VIDEO        = "black" #Video placeholder background
# Fonts
FONT_LABEL      = ("Arial", 14, "bold")
FONT_VALUE      = ("Arial", 18)
FONT_GRAPHIC    = ("Arial", 20)

# Image paths
IMG_ARROW       = "arrow.png"
IMG_ROTATE      = "rotate.png"
IMG_GRIPPER     = "gripper.png"
IMG_VIDEO       = "video.png"
IMG_ARROW_SIZE  = (80, 80)

# Press-and-hold timing (milliseconds)
HOLD_DELAY      = 300    # pause before repeat loop starts
HOLD_INTERVAL   = 100    # interval between repeated signals while held

# Motor names (order matches backend data index)
MOTOR_NAMES     = ["Base", "Height", "Elbow", "Gripper"]
"""
# =============================================================================
# XBOX CONTROLLER KEYBOARD MAP  (via JoyToKey)
# =============================================================================
# JoyToKey translates Xbox inputs into keystrokes.
# Map each Tkinter keysym → robot action string.
#
# Xbox button  │ JoyToKey key  │ Tkinter keysym │ Action
# ─────────────┼───────────────┼────────────────┼──────────────
# Y            │ VK_UP  (0x26) │ Up             │ UP_1
# A            │ VK_DOWN(0x28) │ Down           │ DOWN_1
# B            │ VK_RIGHT(0x27)│ Right          │ RIGHT
# X            │ VK_LEFT(0x25) │ Left           │ LEFT
# POV Up       │ W      (0x57) │ w              │ UP_2
# POV Down     │ S      (0x53) │ s              │ DOWN_2
# Stick1 Left  │ -      (0xBD) │ minus          │ ROTATE_LEFT
# Stick1 Right │ RShift (0xA1) │ Shift_R        │ ROTATE_RIGHT
# RT           │ M      (0x4D  | m              │ GRIPPER
# =============================================================================
"""

KEY_MAP = {
    "Up"      : "UP_1",
    "Down"    : "DOWN_1",
    "Right"   : "RIGHT",
    "Left"    : "LEFT",
    "w"       : "UP_2",
    "s"       : "DOWN_2",
    "minus"   : "ROTATE_LEFT",
    "Shift_R" : "ROTATE_RIGHT",
}
KEY_GRIPPER = "m"   


def setup_keyboard_bindings(root: Tk, controller) -> None:
    """
    Bind Xbox controller keys (translated by JoyToKey) to robot commands.

    BUTTON keys  (Y/A/B/X):  standard press-and-hold — fires on KeyPress,
                              repeats while held, stops on KeyRelease.

    JOYSTICK keys (Stick/POV/Rotate): JoyToKey sends rapid KeyPress bursts
                              but NO KeyRelease when the axis re-centres.
                              These use a "last-fire timeout" pattern:
                              each KeyPress resets a deadline timer; if no
                              new KeyPress arrives within JOYSTICK_TIMEOUT ms
                              the axis is assumed centred and the loop stops.

    GRIPPER key:              single-fire on KeyPress, no repeat.

    Args:
        root      : Tk root window (receives all key events).
        controller: Controller instance whose send_command() is called.
    """

    # Keys whose KeyRelease IS reliable (physical buttons via JoyToKey)
    BUTTON_KEYS   = {"Up", "Down", "Left", "Right"}

    # Keys whose KeyRelease is NOT sent by JoyToKey (analog axes / POV hat)
    JOYSTICK_KEYS = {"w", "s", "minus", "Shift_R"}

    # How long (ms) with no new KeyPress before we treat the axis as centred.
    # Must be longer than JoyToKey's repeat interval but short enough to feel
    # responsive. 150 ms works well for the default JoyToKey repeat rate.
    JOYSTICK_TIMEOUT = 300

    # ── Shared job registry ───────────────────────────────────────────────
    _repeat_jobs  = {}   # action → after() id for the repeat loop. Stores repeating loops (after IDs)
    _timeout_jobs = {}   # action → after() id for the silence detector. Stores timeout detection Loops
    """
    _repeat_jobs ans _timeout_jobs are dictionaries. Flag to study it later
    Stores the IDs returned by Tkinter's after() function.
    _repeat_jobs is for continous repeating actions (holding buttons)
    _timeout_jobs is for detecting inactivity (like stopping when joystick input stops)
    """
    # ── BUTTON hold handlers (reliable KeyRelease) ────────────────────────
    """
    Creates custom handlers for when:
        - A key or button is pressed
        - A key or button is released
    Then:
        - Run once immediately
        - Repeat while you keep holding the button
        - Stop when you release
    """
    def _make_button_handlers(action: str): #Function factory: Create and return functions customized for a specific action. Flag to study later 
        
        def _repeat(): #Creates loop to repeat
            controller.send_command(action) #Sends the command
            _repeat_jobs[action] = root.after(HOLD_INTERVAL, _repeat) #

        def _on_press(event):
            if action in _repeat_jobs:   # OS key-repeat — ignore duplicates
                return
            controller.send_command(action)
            _repeat_jobs[action] = root.after(HOLD_DELAY, _repeat)

        def _on_release(event):
            job = _repeat_jobs.pop(action, None)
            if job:
                root.after_cancel(job)

        return _on_press, _on_release

    # ── JOYSTICK timeout handlers (no reliable KeyRelease) ────────────────
    def _make_joystick_handlers(action: str):
        # Generation counter — the ONLY way to kill an in-flight repeat loop.
        # Incrementing gen[0] makes every pending _repeat() see a stale value
        # and exit immediately, even if root.after() already scheduled it.
        gen = [0]

        def _stop():
            """Called when silence timeout expires — axis is centred."""
            gen[0] += 1                    # invalidate any running repeat loop
            _repeat_jobs.pop(action, None)
            _timeout_jobs.pop(action, None)

        def _make_repeat(my_gen):
            def _repeat():
                if gen[0] != my_gen:       # a _stop() was called — exit
                    return
                controller.send_command(action)
                _repeat_jobs[action] = root.after(HOLD_INTERVAL, _repeat)
            return _repeat

        def _on_press(event):
            # Reset the silence detector on every burst from JoyToKey
            old_timeout = _timeout_jobs.pop(action, None)
            if old_timeout:
                root.after_cancel(old_timeout)
            _timeout_jobs[action] = root.after(JOYSTICK_TIMEOUT, _stop)

            # Start a fresh repeat loop only if none is running this generation
            if action not in _repeat_jobs:
                controller.send_command(action)
                _repeat_jobs[action] = root.after(
                    HOLD_INTERVAL, _make_repeat(gen[0]))

        return _on_press   # no on_release needed

    # ── Register bindings ─────────────────────────────────────────────────
    for keysym, action in KEY_MAP.items():
        if keysym in BUTTON_KEYS:
            press_fn, release_fn = _make_button_handlers(action)
            root.bind(f"<KeyPress-{keysym}>",   press_fn)
            root.bind(f"<KeyRelease-{keysym}>", release_fn)
        elif keysym in JOYSTICK_KEYS:
            press_fn = _make_joystick_handlers(action)
            root.bind(f"<KeyPress-{keysym}>", press_fn)

    # Gripper — single-fire, no repeat
    root.bind(f"<KeyPress-{KEY_GRIPPER}>",
              lambda e: controller.send_command("GRIPPER"))


# =============================================================================
# COMMAND LAYER  (stub — replace with MQTT publish later)
# =============================================================================




# =============================================================================
# PRESS-AND-HOLD BINDING
# =============================================================================

def bind_hold(button: Button, action: str, root: Tk, controller):
    """
    Bind press-and-hold behaviour to a button.
    Sends the command once on press, then repeatedly while held.
    Stops cleanly on release.

    Args:
        button: The Tkinter Button widget to bind.
        action: Command string forwarded to send_command().
        root  : Root window, needed to schedule and cancel after() callbacks.
    """
    _job = [None]   # mutable container so inner functions can cancel the job

    def _repeat() -> None:
        controller.send_command(action)
        _job[0] = root.after(HOLD_INTERVAL, _repeat)

    def _on_press(event) -> None:
        controller.send_command(action)                        # immediate first fire
        _job[0] = root.after(HOLD_DELAY, _repeat)  # start repeat loop

    def _on_release(event) -> None:
        if _job[0] is not None:
            root.after_cancel(_job[0])             # stop the loop
            _job[0] = None

    button.bind("<ButtonPress-1>",   _on_press)
    button.bind("<ButtonRelease-1>", _on_release)


# =============================================================================
# IMAGE LOADER
# =============================================================================

def load_arrow_images(path: str, size: tuple) -> dict:
    """
    Load a single arrow image and return four rotated variants.

    Args:
        path: File path to the base arrow image (pointing right).
        size: (width, height) to resize the image to.

    Returns:
        Dict with keys 'up', 'down', 'left', 'right' as PhotoImage objects.
    """
    base = Image.open(path).resize(size)
    return {
        "up"   : ImageTk.PhotoImage(base.rotate(90,  expand=True)),
        "right": ImageTk.PhotoImage(base),
        "down" : ImageTk.PhotoImage(base.rotate(-90, expand=True)),
        "left" : ImageTk.PhotoImage(base.rotate(180, expand=True)),
    }


def load_misc_images(size: tuple) -> dict:
    """
    Load rotate, gripper, and video placeholder images.

    Args:
        size: (width, height) to resize rotate and gripper images.

    Returns:
        Dict with keys 'rotate_left', 'rotate_right', 'gripper', 'video_raw',
        where 'video_raw' is the raw PIL Image (needed for dynamic resizing).
    """
    rotate_base = Image.open(IMG_ROTATE).resize(size)
    return {
        "rotate_left" : ImageTk.PhotoImage(rotate_base),
        "rotate_right": ImageTk.PhotoImage(
                            rotate_base.transpose(Image.FLIP_LEFT_RIGHT)),
        "gripper"     : ImageTk.PhotoImage(Image.open(IMG_GRIPPER).resize(size)),
        "video_raw"   : Image.open(IMG_VIDEO),   # kept as PIL for resize
    }


# =============================================================================
# COLUMN 1 — MOVEMENT CONTROLS
# =============================================================================

def build_column1(parent: Frame, arrows: dict, misc: dict, root: Tk, controller) -> None:
    """
    Build the movement control column.
    Top half  → directional arrow buttons (two sets: joints 1 and 2).
    Bottom half → rotation and gripper buttons.

    Layout (top, 3 rows x 4 cols):
        [  ][UP1][  ][UP2]
        [LT][  ][RT][  ]
        [  ][DN1][  ][DN2]

    Layout (bottom, 2 rows x 3 cols):
        [ROT_L][  ][ROT_R]
        [  ][GRIP][  ]

    All arrow and rotate buttons use press-and-hold (bind_hold).
    Gripper uses a single click only.

    Args:
        parent: The col1 Frame.
        arrows: Dict from load_arrow_images().
        misc  : Dict from load_misc_images().
        root  : Root window, required by bind_hold for after() scheduling.
    """
    # ── Sub-frames ──────────────────────────────────────────────────────────
    top    = Frame(parent, bg=BG_MAIN)
    bottom = Frame(parent, bg=BG_MAIN)

    top.grid(row=0, column=0, sticky="nsew")
    bottom.grid(row=1, column=0, sticky="nsew")

    parent.rowconfigure(0, weight=1)
    parent.rowconfigure(1, weight=1)
    parent.columnconfigure(0, weight=1)

    # ── Top: directional buttons ─────────────────────────────────────────────
    for r in range(3):
        top.rowconfigure(r, weight=1)
    for c in range(4):
        top.columnconfigure(c, weight=1)

    # Row 0 — up buttons
    btn_up1 = Button(top, image=arrows["up"], bg=BG_MAIN)
    btn_up1.grid(row=0, column=1, sticky="nsew")
    bind_hold(btn_up1, "UP_1", root, controller)

    btn_up2 = Button(top, image=arrows["up"], bg=BG_MAIN)
    btn_up2.grid(row=0, column=3, sticky="nsew")
    bind_hold(btn_up2, "UP_2", root, controller)

    # Row 1 — left / right buttons
    btn_left = Button(top, image=arrows["left"], bg=BG_MAIN)
    btn_left.grid(row=1, column=0, sticky="nsew")
    bind_hold(btn_left, "LEFT", root, controller)

    btn_right = Button(top, image=arrows["right"], bg=BG_MAIN)
    btn_right.grid(row=1, column=2, sticky="nsew")
    bind_hold(btn_right, "RIGHT", root, controller)

    # Row 2 — down buttons
    btn_dn1 = Button(top, image=arrows["down"], bg=BG_MAIN)
    btn_dn1.grid(row=2, column=1, sticky="nsew")
    bind_hold(btn_dn1, "DOWN_1", root, controller)

    btn_dn2 = Button(top, image=arrows["down"], bg=BG_MAIN)
    btn_dn2.grid(row=2, column=3, sticky="nsew")
    bind_hold(btn_dn2, "DOWN_2", root, controller)

    # ── Bottom: rotate and gripper ────────────────────────────────────────────
    for r in range(3):
        bottom.rowconfigure(r, weight=1)
    for c in range(3):
        bottom.columnconfigure(c, weight=1)

    btn_rotl = Button(bottom, image=misc["rotate_left"], bg=BG_MAIN)
    btn_rotl.grid(row=0, column=0, sticky="nsew")
    bind_hold(btn_rotl, "ROTATE_LEFT", root, controller)

    btn_rotr = Button(bottom, image=misc["rotate_right"], bg=BG_MAIN)
    btn_rotr.grid(row=0, column=2, sticky="nsew")
    bind_hold(btn_rotr, "ROTATE_RIGHT", root, controller)

    # Gripper — single click only (no hold)
    Button(bottom, image=misc["gripper"],
           command=lambda: controller.send_command("GRIPPER"),
           bg=BG_MAIN).grid(row=1, column=1, sticky="nsew")


# =============================================================================
# COLUMN 2 — GRAPHICS DISPLAY + VIDEO FEED
# =============================================================================

def build_column2(parent: Frame, video_raw: Image.Image, controller) -> None:
    """
    Build the display column.
    Top 50%  → motor angle plot with Prev / Next navigation.
    Bottom 50% → live video feed (placeholder image, resizes with window).

    Args:
        parent     : The col2 Frame.
        video_raw  : Raw PIL Image used as video placeholder (resized on the fly).
        controller : Controller instance, used to access history for plotting.
    """
    # ── Sub-frames ──────────────────────────────────────────────────────────
    top    = Frame(parent, bg=BG_MAIN)
    bottom = Frame(parent, bg=BG_MAIN)

    # Configure proportions BEFORE gridding children
    parent.rowconfigure(0, weight=1)   # 50% — plot area
    parent.rowconfigure(1, weight=1)   # 50% — video area
    parent.columnconfigure(0, weight=1)

    top.grid(row=0, column=0, sticky="nsew")
    bottom.grid(row=1, column=0, sticky="nsew")

    top.grid_propagate(False)
    bottom.grid_propagate(False)

    # ── Top: graphic carousel ────────────────────────────────────────────────
    top.rowconfigure(0, weight=4)   # display area expands
    top.rowconfigure(1, weight=1)   # buttons stay compact
    top.columnconfigure(0, weight=1)

    # Display area
    display_frame = Frame(top, bg=BG_DISPLAY)
    display_frame.grid(row=0, column=0, sticky="nsew")
    display_frame.rowconfigure(0, weight=1)
    display_frame.columnconfigure(0, weight=1)

    current_index = [0]   # mutable container so inner functions can write to it

    # ── Matplotlib figure for motor angle plotting ──
    fig = Figure(figsize=(5, 3), dpi=100)
    ax  = fig.add_subplot(111)
    ax.set_title(MOTOR_NAMES[current_index[0]])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (°)")
    ax.grid(True)

    canvas = FigureCanvasTkAgg(fig, master=display_frame)
    canvas.draw()
    canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    # Navigation callbacks
    def show_motor(offset: int) -> None:
        current_index[0] = (current_index[0] + offset) % len(MOTOR_NAMES)
        ax.clear()
        motor_name = MOTOR_NAMES[current_index[0]]
        ax.set_title(motor_name)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (°)")
        ax.grid(True)

        # Draw latest data if available
        if controller.history:
            hist = controller.history[motor_name]
            if hist["times"]:
                # relative time (seconds)
                times = [t - hist["times"][0] for t in hist["times"]]
                values = list(hist["values"])
                ax.plot(times, values, color="blue")
        canvas.draw()

    # Navigation buttons
    nav_frame = Frame(top, bg=BG_MAIN)
    nav_frame.grid(row=1, column=0, sticky="nsew")
    nav_frame.columnconfigure(0, weight=1)
    nav_frame.columnconfigure(1, weight=1)

    Button(nav_frame, text="⬅ Prev",
           command=lambda: show_motor(-1))\
        .grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
    Button(nav_frame, text="Next ➡",
           command=lambda: show_motor(+1))\
        .grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

    # ── Bottom: video feed ───────────────────────────────────────────────────
    bottom.rowconfigure(0, weight=1)
    bottom.columnconfigure(0, weight=1)

    video_label = Label(bottom, bg=BG_VIDEO)
    video_label.grid(row=0, column=0, sticky="nsew")

    def on_resize(event) -> None:
        """Rescale the video placeholder whenever the frame is resized."""
        resized   = video_raw.resize((event.width, event.height))
        new_photo = ImageTk.PhotoImage(resized)
        video_label.config(image=new_photo)
        video_label.image = new_photo   # prevent garbage collection

    bottom.bind("<Configure>", on_resize)

    # Real-time update callback (controller will call this)
    def plot_update():
        motor_name = MOTOR_NAMES[current_index[0]]
        hist = controller.history[motor_name]
        if hist["times"]:
            ax.clear()
            ax.set_title(motor_name)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Angle (°)")
            ax.grid(True)
            times = [t - hist["times"][0] for t in hist["times"]]
            values = list(hist["values"])
            ax.plot(times, values, color="blue")
            canvas.draw()

    # Store callback in frame so controller can access it
    parent.plot_update_callback = plot_update

# =============================================================================
# COLUMN 3 — MOTOR ANGLE DISPLAY
# =============================================================================

def build_column3(parent: Frame) -> list:
    """
    Build the motor information column.
    Alternating rows: motor name label → current angle value box.

    Row layout (8 rows total):
        0 → "Base:"      label
        1 → angle value  box
        2 → "Height:"    label
        3 → angle value  box
        ...

    Args:
        parent: The col3 Frame.

    Returns:
        List of StringVar objects (one per motor, same order as MOTOR_NAMES).
        Use motor_vars[i].set("45.00°") from the backend to update the UI.
    """
    # Label rows (even) get weight=1, value rows (odd) get weight=2
    for r in range(8):
        parent.rowconfigure(r, weight=1 if r % 2 == 0 else 2)
    parent.columnconfigure(0, weight=1)

    motor_vars = []
    

    for i, name in enumerate(MOTOR_NAMES):
        row_label = i * 2       # 0, 2, 4, 6
        row_value = i * 2 + 1   # 1, 3, 5, 7

        # Motor name label
        Label(parent,
              text=f"{name}:",
              bg=BG_MAIN,
              font=FONT_LABEL,
              anchor="w",
              padx=10)\
            .grid(row=row_label, column=0, sticky="nsew")

        # Angle value box (backend updates this via var.set())
        var = StringVar(value="0.00°")
        motor_vars.append(var)

        Label(parent,
              textvariable=var,
              bg=BG_DISPLAY,
              font=FONT_VALUE,
              relief="sunken",
              anchor="center")\
            .grid(row=row_value, column=0, sticky="nsew", padx=20, pady=5)

    return motor_vars


# =============================================================================
# MAIN — WINDOW + ROOT GRID
# =============================================================================

def main():
    root = Tk()
    root.geometry(WINDOW_SIZE)
    root.title(WINDOW_TITLE)

    # ── Frames ────────────────────────────────────────────────────────────
    col1 = Frame(root, bg="red")
    col2 = Frame(root, bg="green")
    col3 = Frame(root, bg="blue")

    for col_index, frame in enumerate([col1, col2, col3]):
        frame.grid(row=0, column=col_index, sticky="nsew")
        frame.grid_propagate(False)
        root.columnconfigure(col_index, weight=1, uniform="equal")

    root.rowconfigure(0, weight=1)

    # ── Load assets ───────────────────────────────────────────────────────
    arrows = load_arrow_images(IMG_ARROW, IMG_ARROW_SIZE)
    misc   = load_misc_images(IMG_ARROW_SIZE)
    root._images = {**arrows, **{k: v for k, v in misc.items()
                                 if isinstance(v, ImageTk.PhotoImage)}}

    # ── Build col3 FIRST to get motor_vars ───────────────────────────────
    motor_vars = build_column3(col3)

    # ── Create controller NOW that motor_vars exists ──────────────────────
    mqtt_client = MQTTClient(
        broker_ip="10.246.50.24",
        port=1883,
        username="Frii",
        password="12345678",
        on_message_callback=None
    )
    controller = Controller(mqtt_client, motor_vars, root)
    mqtt_client.on_message_callback = controller.handle_mqtt_message
    mqtt_client.connect()

    # ── Build col2 (needs controller for plot) ────────────────────────────
    build_column2(col2, misc["video_raw"], controller)
    controller.plot_update_callback = col2.plot_update_callback

    # ── Build col1 LAST (needs controller for button commands) ────────────
    build_column1(col1, arrows, misc, root, controller)

    # ── Xbox controller support (via JoyToKey keyboard translation) ───────
    setup_keyboard_bindings(root, controller)

    root.mainloop()

    


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    main()