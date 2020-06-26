from PointCloudTools import ScanObject





print("-------------------------------------------------------------------------")
print("Test Function for using the 3D Scan Hardware with a resolution of 89x89")
print("-------------------------------------------------------------------------")

LiveScan = ScanObject("Living_Room_low", 89)
print("-------------------------------------------------------------------------")
LiveScan.lidarSensorRead()
print("-------------------------------------------------------------------------")
LiveScan._transformData()
LiveScan.saveScan()
LiveScan.clustering(8)
LiveScan.plotpointclouddata()







