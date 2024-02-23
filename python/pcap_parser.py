#import open3d as o3d
import numpy as np
from ouster import client, pcap
import os
#import matplotlib.pyplot as plt

def getScansFromPcapFiles(sourcePcapPath, sourceJsonPath, numScans=-1):
    
    with open(sourceJsonPath, 'r') as f:
        metadata = client.SensorInfo(f.read())

    source = pcap.Pcap(sourcePcapPath, metadata)
    xyzlut = client.XYZLut(metadata)

    scans = client.Scans(source)
    
    xyzScans = []
    reflectivityScans = []
    count = 0

    for scan in scans:
        
        if numScans == count: break
        
        ranges_staggered = scan.field(client.ChanField.RANGE)
        ranges = client.destagger(source.metadata, ranges_staggered)
        xyz = np.array(xyzlut(ranges))
        reflectivity_staggered = np.array(scan.field(client.ChanField.REFLECTIVITY))
        reflectivity = client.destagger(source.metadata, reflectivity_staggered)
        
        xyzScans.append(xyz)
        reflectivityScans.append(reflectivity)
        
        count += 1
    
    return xyzScans, reflectivityScans

def getScansFromPcapFolder(folderPath, numScans=-1):
    
    pcapSourcePath = ''
    metadataPath = ''
    directory = os.fsencode(folderPath)
    
    for file in os.listdir(directory):
        filename = os.fsdecode(file)
        if filename.endswith(".json"):
            metadataPath = folderPath + '/' + filename
        elif filename.endswith(".pcap"): 
            pcapSourcePath = folderPath + '/' + filename
         
    return getScansFromPcapFiles(pcapSourcePath, metadataPath, numScans)