#Author-
#Description-

import sys, os, threading, json, subprocess
pyt = os.path.join(os.path.split(os.__file__)[0], '..\\python')
numpy_dir = os.path.dirname(os.__file__) + '\\site-packages\\numpy'
pynput_dir = os.path.dirname(os.__file__) + '\\site-packages\\pynput'

if os.path.isdir(numpy_dir) == False:
    process = subprocess.Popen('cmd /c ' + pyt + ' -m pip install numpy')
    process.wait()
if os.path.isdir(pynput_dir) == False:
    process = subprocess.Popen('cmd /c ' + pyt + ' -m pip install pynput')
    process.wait()

import numpy as np
from pynput import keyboard
from math import degrees, radians, cos, sin
import adsk.core, adsk.fusion, adsk.cam, traceback, Command


app = None
ui = adsk.core.UserInterface.cast(None)
# Get the palette that represents the TEXT COMMANDS window. 

handlers = []
stopFlag = None
custom_event_identifier = 'F360 Numpad Views'
customEvent = None
o_ReferenceFrame = None


# Custom thread, ADSK code should not be executed here
# or in any of the functions called from here
class CustomThread(threading.Thread):
    def __init__(self, event):
        threading.Thread.__init__(self)
        self.stopped = event
        self.setDaemon(True)
    def run(self):
        
        with keyboard.Listener(on_press=f_alt_down) as listener:
            listener.join() # wait for alt_l
        with keyboard.Listener(on_press=f_key_loop_down, on_release=f_key_loop_up) as listener:
            listener.join()

        while self.stopped._flag == False:
            if listener.running == True:
                continue
            else:
                with keyboard.Listener(on_press=f_alt_down) as listener:
                    listener.join() # wait for alt_l
                with keyboard.Listener(on_press=f_key_loop_down, on_release=f_key_loop_up) as listener:
                    listener.join() # wait for numpad key down or alt_l up


def f_alt_down(a_key):
    if a_key == keyboard.Key.alt_l:
        print(str(a_key) + ' pressed...')
        return False


def f_key_loop_down(a_key):
    global app
    if a_key != keyboard.Key.alt_l:
        if hasattr(a_key, 'vk') and 96 <= a_key.vk <= 105:
            print(str(keyboard.Key.alt_l) + ' held and ' + str(a_key.vk) + ' pressed')
            if str(a_key.vk) == str(97):
                event_packet = json.dumps({ 'translate':[-1,-1,0], 'rotate' : [0,0,0]}) # Numpad1
            elif str(a_key.vk) == str(98):
                event_packet = json.dumps({ 'translate':[0,-1,-1], 'rotate' : [90,90,90]}) # Numpad2
            elif str(a_key.vk) == str(99):
                event_packet = json.dumps({ 'translate':[1,-1,0],  'rotate' : [0,0,0]}) # Numpad3
            elif str(a_key.vk) == str(100):
                event_packet = json.dumps({ 'translate':[-1,0,-1], 'rotate' : [270,0,0]}) # Numpad4
            elif str(a_key.vk) == str(101):
                event_packet = json.dumps({ 'translate':[0,0,0],   'rotate' : [0,0,0]}) # Numpad5
            elif str(a_key.vk) == str(102):
                event_packet = json.dumps({ 'translate':[1,0,-1],  'rotate' : [0,0,0]}) # Numpad6
            elif str(a_key.vk) == str(103):
                event_packet = json.dumps({ 'translate':[-1,1,0],  'rotate' : [0,0,0]}) # Numpad7
            elif str(a_key.vk) == str(104):
                event_packet = json.dumps({ 'translate':[0,1,-1],  'rotate' : [0,0,0]}) # Numpad8
            elif str(a_key.vk) == str(105):
                event_packet = json.dumps({ 'translate':[1,1,0],   'rotate' : [0,0,0]}) # Numpad9
            elif str(a_key.vk) == str(96):
                event_packet = json.dumps({})  # Numpad0
            app.fireCustomEvent(custom_event_identifier, event_packet)
            return False

def f_key_loop_up(a_key):
    if a_key == keyboard.Key.alt_l:
        print('...' + str(a_key) + ' released.')
        return False




# The event handler that responds to the custom event being fired.
# ADSK code can be executed here, or in any functions called from here.
class ThreadEventHandler(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        global o_ReferenceFrame, ui
        received_packet = json.loads(args.additionalInfo)
        # Make sure a command isn't running before changes are made.
        if ui.activeCommand != 'SelectCommand':
            ui.commandDefinitions.itemById('SelectCommand').execute()
        if len(received_packet) == 2:
            cam_change(received_packet['translate'], received_packet['rotate'], o_ReferenceFrame)
        else:
            o_ReferenceFrame = getReferenceFrame()
            # Create a new document with parametric design
            doc = app.activeDocument
            product = app.activeProduct
    
            design = adsk.fusion.Design.cast(product)
            design.designType = adsk.fusion.DesignTypes.DirectDesignType
            # Get the root component of active design
            rootComp = design.rootComponent
            for i in rootComp.customGraphicsGroups:
                for x in i:
                    x.deleteMe()
                i.deleteMe()
            
            # Get TemporaryBRepManager
            tempBrepMgr = adsk.fusion.TemporaryBRepManager.get()
    
            # Create wire from curves
            wireBody, edgeMap = CreateWCS(o_ReferenceFrame)
            DrawWCS(wireBody,rootComp)


def getReferenceFrame():
    global app, ui
    textPalette = ui.palettes.itemById('TextCommands')
    # Make sure the palette is visible.
    if textPalette.isVisible == False:
        textPalette.isVisible = True
    

    world_origin_WCS = adsk.core.Point3D.create(0,0,0)
    world_x_unit_WCS = adsk.core.Vector3D.create(1,0,0)
    world_y_unit_WCS = adsk.core.Vector3D.create(0,1,0)
    world_z_unit_WCS = adsk.core.Vector3D.create(0,0,1)

    o_Camera = app.activeViewport.camera
    local_origin_WCS = o_Camera.target
    local_eye_WCS = o_Camera.eye
    local_up_WCS = o_Camera.upVector

    origins_vector_WCS = world_origin_WCS.vectorTo(local_origin_WCS)
    local_origin2eye_WCS = local_origin_WCS.vectorTo(local_eye_WCS)

    local_x_WCS = local_up_WCS.crossProduct(local_origin2eye_WCS)
    local_y_WCS = local_up_WCS
    local_z_WCS = local_origin2eye_WCS
    view_distance = round(local_z_WCS.length,2)

    local_x_unit_WCS = adsk.core.Vector3D.create(
                        round(local_x_WCS.x,2)/view_distance,
                        round(local_x_WCS.y,2)/view_distance,
                        round(local_x_WCS.z,2)/view_distance)

    local_y_unit_WCS = adsk.core.Vector3D.create(
                        round(local_y_WCS.x,2),
                        round(local_y_WCS.y,2),
                        round(local_y_WCS.z,2))

    local_z_unit_WCS = adsk.core.Vector3D.create(
                        round(local_z_WCS.x,2)/view_distance,
                        round(local_z_WCS.y,2)/view_distance,
                        round(local_z_WCS.z,2)/view_distance)

    axis_map = ['ix','iy','iz','jx','jy','jk','kx','ky','kz']

    angle_ix = round(degrees(local_x_unit_WCS.angleTo(world_x_unit_WCS)),2)
    angle_iy = round(degrees(local_x_unit_WCS.angleTo(world_y_unit_WCS)),2)
    angle_iz = round(degrees(local_x_unit_WCS.angleTo(world_z_unit_WCS)),2)
    angle_jx = round(degrees(local_y_unit_WCS.angleTo(world_x_unit_WCS)),2)
    angle_jy = round(degrees(local_y_unit_WCS.angleTo(world_y_unit_WCS)),2)
    angle_jz = round(degrees(local_y_unit_WCS.angleTo(world_z_unit_WCS)),2)
    angle_kx = round(degrees(local_z_unit_WCS.angleTo(world_x_unit_WCS)),2)
    angle_ky = round(degrees(local_z_unit_WCS.angleTo(world_y_unit_WCS)),2)
    angle_kz = round(degrees(local_z_unit_WCS.angleTo(world_z_unit_WCS)),2)

    local_origin_WCS =    [round(item,2) for item in local_origin_WCS.asArray()]
    local_eye_WCS =       [round(item,2) for item in local_eye_WCS.asArray()]
    origins_vector_WCS =  [round(item,2) for item in origins_vector_WCS.asArray()]
    local_x_unit_WCS =    [round(item,2) for item in local_x_unit_WCS.asArray()]
    local_y_unit_WCS =    [round(item,2) for item in local_y_unit_WCS.asArray()]
    local_z_unit_WCS =    [round(item,2) for item in local_z_unit_WCS.asArray()]

    local_x_unit_WCS = noNegativeZero(local_x_unit_WCS)
    local_y_unit_WCS = noNegativeZero(local_y_unit_WCS)
    local_z_unit_WCS = noNegativeZero(local_z_unit_WCS)

    local_origin_WCS    = noNegativeZero(local_origin_WCS)
    local_eye_WCS       = noNegativeZero(local_eye_WCS)
    origins_vector_WCS  = noNegativeZero(origins_vector_WCS)

    print('### Anchored ###')
    textPalette.writeText('### Anchored ###')
    packet = {
        'o_Camera'          :o_Camera,
        'view_distance'     :view_distance,
        'local_origin_WCS'  :local_origin_WCS,
        'local_eye_WCS'     :local_eye_WCS,
        'origins_vector_WCS':origins_vector_WCS,
        'local_x_unit_WCS'  :local_x_unit_WCS,
        'local_y_unit_WCS'  :local_y_unit_WCS,
        'local_z_unit_WCS'  :local_z_unit_WCS,
        'angle_ix'          :angle_ix,
        'angle_iy'          :angle_iy,
        'angle_iz'          :angle_iz,
        'angle_jx'          :angle_jx,
        'angle_jy'          :angle_jy,
        'angle_jz'          :angle_jz,
        'angle_kx'          :angle_kx,
        'angle_ky'          :angle_ky,
        'angle_kz'          :angle_kz,}

    for key, value in packet.items():
        print('\t\t ' + str(key) + ': ' + str(value))
        textPalette.writeText('\t\t ' + str(key) + ': ' + str(value))

    custom_line = adsk.fusion.CustomGraphicsLines
    custom_line_start = adsk.fusion.CustomGraphicsCoordinates.create([0,0,0])
    custom_line_end = adsk.fusion.CustomGraphicsCoordinates.create([0,0,100])
    custom_line.coordinates = custom_line_end
    return packet




def noNegativeZero(args):
    dummy_list = []
    for i in args:
        if i == -0.0:
            dummy_list.append(0.0)
        else:
            dummy_list.append(i)
    return dummy_list




def cam_change(a_translate_LCS, a_rotate_LCS, a_ReferenceFrame):
    global app
    
    local_origin_WCS = adsk.core.Point3D.create(
            a_ReferenceFrame['local_origin_WCS'][0],
            a_ReferenceFrame['local_origin_WCS'][1],
            a_ReferenceFrame['local_origin_WCS'][2])

    local_eye_WCS = adsk.core.Point3D.create(
            a_ReferenceFrame['local_eye_WCS'][0],
            a_ReferenceFrame['local_eye_WCS'][1],
            a_ReferenceFrame['local_eye_WCS'][2])

    local_y_unit_WCS = np.array([
            [a_ReferenceFrame['local_y_unit_WCS'][0]],
            [a_ReferenceFrame['local_y_unit_WCS'][1]],
            [a_ReferenceFrame['local_y_unit_WCS'][2]]
    ])

    print('local_y_unit_WCS')
    print(local_y_unit_WCS)

    old_z_vector = local_origin_WCS.vectorTo(local_eye_WCS)
    print('old_z_vector')
    print(str(old_z_vector.asArray()))

    transform_matrix_WCS = np.array([
            [round(cos(radians(a_ReferenceFrame['angle_ix'])),2), round(cos(radians(a_ReferenceFrame['angle_iy'])),2), round(cos(radians(a_ReferenceFrame['angle_iz'])),2)],
            [round(cos(radians(a_ReferenceFrame['angle_jx'])),2), round(cos(radians(a_ReferenceFrame['angle_jy'])),2), round(cos(radians(a_ReferenceFrame['angle_jz'])),2)],
            [round(cos(radians(a_ReferenceFrame['angle_kx'])),2), round(cos(radians(a_ReferenceFrame['angle_ky'])),2), round(cos(radians(a_ReferenceFrame['angle_kz'])),2)]
            ])

    print('transform_matrix_WCS')
    print(transform_matrix_WCS)

    print('transform_transpose')
    print(np.transpose(transform_matrix_WCS))

    local_translate_matrix_LCS = np.array([
            [a_translate_LCS[0] * a_ReferenceFrame['view_distance']],
            [a_translate_LCS[1] * a_ReferenceFrame['view_distance']],
            [a_translate_LCS[2] * a_ReferenceFrame['view_distance']]
            ])
    print('local_translate_matrix_LCS')
    print(local_translate_matrix_LCS)

    
    local_translate_matrix_WCS = np.transpose(transform_matrix_WCS) * local_translate_matrix_LCS

    print('local_translate_matrix_WCS')
    print(local_translate_matrix_WCS)

    local_translate_vector_WCS = adsk.core.Vector3D.create(
            local_translate_matrix_WCS.item((0,0)) + local_translate_matrix_WCS.item((0,1)) + local_translate_matrix_WCS.item((0,2)),
            local_translate_matrix_WCS.item((1,0)) + local_translate_matrix_WCS.item((1,1)) + local_translate_matrix_WCS.item((1,2)),
            local_translate_matrix_WCS.item((2,0)) + local_translate_matrix_WCS.item((2,1)) + local_translate_matrix_WCS.item((2,2))
            )

    print('local_translate_vector')
    print(str(local_translate_vector_WCS.asArray()))

    new_local_eye_WCS = adsk.core.Point3D.create(
            local_eye_WCS.x + local_translate_vector_WCS.x,
            local_eye_WCS.y + local_translate_vector_WCS.y,
            local_eye_WCS.z + local_translate_vector_WCS.z
            )
    '''
    phi     = radians(a_rotate_LCS[0])
    theta   = radians(a_rotate_LCS[1])
    psi     = radians(a_rotate_LCS[2])

    rotation_matrix = np.array([
            [round( cos(theta)*cos(psi), 2), round( (-cos(phi)*sin(psi)) + (sin(theta)*cos(psi)), 2), round( sin(phi)*sin(psi) + (cos(phi)*sin(theta)*cos(psi)), 2) ],
            [round( cos(theta)*sin(psi), 2), round( (cos(phi)*cos(psi)) + (sin(phi)*sin(theta)*sin(psi)), 2), round( (-sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi)), 2) ],
            [round( -sin(theta), 2), round( sin(phi)*cos(theta), 2), round( cos(phi)*cos(theta), 2)]
        ])

    print('rotation_matrix')
    print(rotation_matrix)

    new_up_matrix = rotation_matrix * local_y_unit_WCS
    
    print('new_up_matrix')
    print(new_up_matrix)

    new_up_vector = adsk.core.Vector3D.create(
            new_up_matrix.item((0,0)) + new_up_matrix.item((0,1)) + new_up_matrix.item((0,2)),
            new_up_matrix.item((1,0)) + new_up_matrix.item((1,1)) + new_up_matrix.item((1,2)),
            new_up_matrix.item((2,0)) + new_up_matrix.item((2,1)) + new_up_matrix.item((2,2))
            )

    print('new_up_vector')
    print(str(new_up_vector.asArray()))
    '''
    new_z_vector = local_origin_WCS.vectorTo(new_local_eye_WCS)
    new_x_vector = old_z_vector.crossProduct(new_z_vector)
    new_y_vector = new_z_vector.crossProduct(new_x_vector)
    o_Camera = a_ReferenceFrame['o_Camera']
    o_Camera.eye = new_local_eye_WCS
    o_Camera.target = local_origin_WCS
    o_Camera.upVector = new_y_vector

    print('new xyz vector')
    print(str(new_x_vector.asArray()))
    print(str(new_y_vector.asArray()))
    print(str(new_z_vector.asArray()))

    app.activeViewport.camera = o_Camera
    ##o_Camera.isSmoothTransition = False
    app.activeViewport.refresh()

def CreateWCS(ao_ReferenceFrame):
    # Get TemporaryBRepManager
    tempBrepMgr = adsk.fusion.TemporaryBRepManager.get() 
    
    local_origin = ao_ReferenceFrame['local_origin_WCS']
    origin_start = adsk.core.Point3D.create(local_origin[0], local_origin[1], local_origin[2])
    osa = origin_start.asArray()
    x_end = adsk.core.Point3D.create(osa[0] + 10.0, osa[1] + 0.0, osa[2] + 0.0)
    y_end = adsk.core.Point3D.create(osa[0] + 0.0, osa[1] + 10.0, osa[2] + 0.0)
    z_end = adsk.core.Point3D.create(osa[0] + 0.0, osa[1] + 0.0, osa[2] + 10.0)
    x_axis = adsk.core.Line3D.create(origin_start,x_end)
    y_axis = adsk.core.Line3D.create(origin_start,y_end)
    z_axis = adsk.core.Line3D.create(origin_start,z_end)
    
    curves = []
    curves.append(x_axis)
    curves.append(y_axis)
    curves.append(z_axis)
    
    # Create wire from curves
    wireBody, edgeMap = tempBrepMgr.createWireFromCurves(curves)
    return wireBody, edgeMap
    

def DrawWCS(arg1, arg2):

    # Creates a body from multiple wires that all lie within the same plane
    wireBodies = []
    wireBodies.append(arg1)
    # Display the helix edges
    rootComp = arg2
    group = rootComp.customGraphicsGroups.add()
    for edge in arg1.edges:
        group.addCurve(edge.geometry)
    return


def run(context):
    global app, ui, customEvent, stopFlag
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        textPalette = ui.palettes.itemById('TextCommands')
        # Make sure the palette is visible.
        if textPalette.isVisible == False:
            textPalette.isVisible = True
        textPalette.writeText('>>> Start addin')

        # DebugCommands.ListCommandDefinitions << Text command to get all available commands
        # Command.Start('Commands.Start ShowTextCommandsCommand')


        # Register the custom event and connect the handler.
        customEvent = app.registerCustomEvent(custom_event_identifier)
        custom_thread_handler = ThreadEventHandler()
        customEvent.add(custom_thread_handler)
        handlers.append(custom_thread_handler)

        # Create a new thread for the other processing.
        stopFlag = threading.Event()
        workerThread = CustomThread(stopFlag)
        workerThread.start()
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    global app, ui
    textPalette = ui.palettes.itemById('TextCommands')

    # Make sure the palette is visible.
    if textPalette.isVisible == False:
        textPalette.isVisible = True
    try:
        if handlers.count:
            customEvent.remove(handlers[0])
        stopFlag.set()
        app.unregisterCustomEvent(custom_event_identifier)
        keyboard.Listener.stop
        textPalette.writeText('Stop addin <<<')
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))