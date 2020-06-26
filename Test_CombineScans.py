from PointCloudTools import ScanObject


print("---------------------------------------------------------------------------------------------------------------")
print("Test Function for combining multiple Point Clouds to one ")
print("---------------------------------------------------------------------------------------------------------------")


print("Bedroom")
print("---------------------------------------------------------------------------------------------------------------")
BedRoomScan = ScanObject("BedRoom", 178)
print("---------------------------------------------------------------------------------------------------------------")
BedRoomScan.loadPointCloudData("PointCloudData_BedRoom.txt")

print("Living Room")
print("---------------------------------------------------------------------------------------------------------------")
LivingRoom = ScanObject("LivingRoom", 178)
LivingRoom.loadPointCloudData("PointCloudData_LivingRoom.txt")


print("---------------------------------------------------------------------------------------------------------------")
print("Connect Living Room and Bed room")
print("---------------------------------------------------------------------------------------------------------------")

LivingRoom.addscanobjects(BedRoomScan, 550, 0, 20)
print(LivingRoom)
LivingRoom.plotpointclouddata()
