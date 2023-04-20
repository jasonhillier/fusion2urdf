# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019
Modified on Sun Jan 17 2021

@author: syuntoku
@author: spacemaster85
"""

import adsk
import adsk.core
import adsk.fusion
import os.path
import re
from xml.etree import ElementTree
from xml.dom import minidom
from distutils.dir_util import copy_tree
import fileinput
import sys
import json

_debugFlag = False
_showProgress = False
#
_progressBar = None
_progressCount = 0
_json_config = None
_log_path = None

def loadConfig(path, package_name):
    global _json_config
    global _log_path

    _log_path = path
    os.remove(_log_path + '\\fusion2urdf.log')

    with open(path + '\\fusion2urdf_config.json', 'r') as c_file:
        _json_config = json.load(c_file)
    
    if package_name in _json_config:
        _json_config = _json_config[package_name]
        for key in _json_config:
            showMessage("config key: " + key)
    else:
        logMessage("Config not found for %s" % package_name)

def logMessage(message):
    if _log_path is None: return
    with open(_log_path + '\\fusion2urdf.log', 'a') as log_file:
        log_file.write(message + "\n")
    
def showProgress(message, val=0, max=0):
    global _progressBar
    global _progressCount

    app = adsk.core.Application.get()
    ui = app.userInterface

    logMessage(message)

    if _showProgress is False: return

    if _progressBar is None:
        _progressBar = ui.createProgressDialog()
        _progressBar.show("v1.0", message, 0, 10)
    if _progressBar.isShowing:
        _progressBar.message = message
        _progressBar.progress = ++_progressCount
    else:
        _progressBar.show("v1.0", message, 0, 10)

def endProgressBar():
    if _progressBar is not None: _progressBar.hide()

def showMessage(message, type=None):
    global _debugFlag

    app = adsk.core.Application.get()
    ui = app.userInterface

    #always write all messages to log file
    logMessage(message)

    if _debugFlag is True or type is not None: ui.messageBox(message)

#todo: make recursive
def override_object(object, newDef):
    for key in newDef:
        if key in object: object[key] = newDef[key]

def get_occ_recursive(nodes, allOccs):
    for node in nodes:
        allOccs.append(node)
        if node.component and node.component.occurrences:
            get_occ_recursive(node.component.occurrences, allOccs)

def export_stl(_app, save_dir):
    """
    export stl files into "sace_dir/"


    Parameters
    ----------
    _app: adsk.core.Application.get()
    save_dir: str
        directory path to save
    """

    def traverse( occ):
    # recursive method to get all bodies from components and sub-components
        body = adsk.fusion.BRepBody.cast(None)
        liste = []
        if occ.childOccurrences and occ.isLightBulbOn:
            for child in occ.childOccurrences:
                liste = liste + traverse(child)
        if occ.isLightBulbOn:   
            liste = liste + [body for body in occ.bRepBodies if body.isLightBulbOn and occ.component.isBodiesFolderLightBulbOn]
        return liste


    des: adsk.fusion.Design = _app.activeProduct
    root: adsk.fusion.Component = des.rootComponent

    showBodies = []
    body = adsk.fusion.BRepBody.cast(None)
    if root.isBodiesFolderLightBulbOn:
        lst = [body for body in root.bRepBodies if body.isLightBulbOn]
        if len(lst) > 0:
            showBodies.append(['root', lst])

        occ = adsk.fusion.Occurrence.cast(None)
        allOccs = []
        get_occ_recursive(root.allOccurrences, allOccs)

        for occ in allOccs:
            if not occ.assemblyContext and occ.isLightBulbOn:
                lst = [body for body in occ.bRepBodies if body.isLightBulbOn and occ.component.isBodiesFolderLightBulbOn]
                if occ.childOccurrences:
                    for child in occ.childOccurrences:
                        lst = lst + traverse(child)
                if len(lst) > 0:
                    showBodies.append([occ.name, lst])

        # get clone body
        found_base = False
        tmpBrepMng = adsk.fusion.TemporaryBRepManager.get()
        tmpBodies = []
        for name, bodies in showBodies:
            lst = [tmpBrepMng.copy(body) for body in bodies]
            if len(lst) > 0:
                if 'base_link' in name:
                    if found_base:
                        continue
                    else:
                        found_base = True
                logMessage("cloning body %s" % (name))
                tmpBodies.append([name, lst])

        # create export Doc - DirectDesign
        fusionDocType = adsk.core.DocumentTypes.FusionDesignDocumentType
        expDoc: adsk.fusion.FusionDocument = _app.documents.add(fusionDocType)
        expDes: adsk.fusion.Design = expDoc.design
        expDes.designType = adsk.fusion.DesignTypes.DirectDesignType

        # get export rootComponent
        expRoot: adsk.fusion.Component = expDes.rootComponent

        # paste clone body
        mat0 = adsk.core.Matrix3D.create()
        for name, bodies in tmpBodies:
            occ = expRoot.occurrences.addNewComponent(mat0)
            comp = occ.component
            comp.name = name
            for body in bodies:
                comp.bRepBodies.add(body)

        # export stl
        try:
            os.mkdir(save_dir + '/meshes')
        except:
            pass
        exportFolder = save_dir + '/meshes'

        exportMgr = des.exportManager

        for occ in expRoot.allOccurrences:
            if "base_link" in occ.component.name:
                expName = "base_link"
            else:
                expName = re.sub('[ :()]', '_', occ.component.name)
            expPath = os.path.join(exportFolder, '{}.stl'.format(expName))
            stlOpts = exportMgr.createSTLExportOptions(occ, expPath)
            logMessage("exporting mesh %s" % (expPath))
            exportMgr.execute(stlOpts)

        # remove export Doc
        expDoc.close(False)

def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog'

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]


    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                          -x*y, -y*z, -x*z]
    return [round(i - mass*t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element


    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(save_dir, package_dir):
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass
    try:
        os.mkdir(save_dir + '/urdf')
    except:
        pass
    copy_tree(package_dir, save_dir)


def update_cmakelists(save_dir, package_name):
    file_name = save_dir + '/CMakeLists.txt'

    for line in fileinput.input(file_name, inplace=True):
        if 'project(fusion2urdf)' in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)

def update_ros2_launchfile(save_dir, package_name):
    file_name = save_dir + '/launch/robot_description.launch.py'

    for line in fileinput.input(file_name, inplace=True):
        if 'fusion2urdf' in line:
            sys.stdout.write(line.replace('fusion2urdf', package_name))
        else:
            sys.stdout.write(line)

def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " +
                             package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
