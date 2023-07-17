from qgis.gui import *
from qgis.core import *
from PyQt5.QtCore import *
from qgis.PyQt.QtWidgets import QAction, QMainWindow
import pandas as pd
import datetime

from rclpy.node import Node
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from threading import Thread

from px4_msgs.msg import VehicleGlobalPosition


def add_layer_toproject(layer,namety):
    """
    """
    if not layer.isValid():
        print(f"{namety} Layer failed to load!")
    else:
        QgsProject.instance().addMapLayer(layer)

def gen_lidarlayer():
    """
    """
    lidar_layer=QgsPointCloudLayer("/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/ui/layers_custom/PTS_LAMB93_IGN69_0925_6326.las","lidar", "pdal")
    # lidar_layer=QgsPointCloudLayer("/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/ui/layers_custom/dom04_714_5323_1_by.las","lidar", "pdal")
    # crs = lidar_layer.crs()
    # crs.createFromId(25832) 
    # lidar_layer.setCrs(crs)
    return lidar_layer

def gen_zonelayer():
    """
    """
    zone_layer=QgsVectorLayer("/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/ui/layers_custom/testzone.shp","monitoringZone", "ogr")
    zone_layer.loadNamedStyle('/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/ui/style/zoneProbaGradua.qml')
    features=zone_layer.getFeatures()
    layer_provider=zone_layer.dataProvider()
    zone_layer.startEditing()
    snew={'w0':1,'w1':0,'w2':0,'w3':1}
    index_field_state=zone_layer.fields().indexFromName('state')
    for f in features:
        valuestate=snew[f['name']]
        print(valuestate)
        layer_provider.changeAttributeValues({f.id():{index_field_state:valuestate}})
    zone_layer.commitChanges()
    return zone_layer



def gen_dronelayer():
    """
    """
    drone_layers = QgsVectorLayer("Point?crs=EPSG:4326", "temporary_points", "memory")
    drone_layers.loadNamedStyle('/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/ui/style/drones.qml')
    pr = drone_layers.dataProvider()
    pr.addAttributes( [ QgsField("drone", QVariant.String),
                    QgsField("X",  QVariant.Int),
                    QgsField("Y",  QVariant.Int),
                   QgsField("date",QVariant.DateTime) ] ) 
    drone_layers.updateFields()              
    drone_layers.commitChanges()
    tempo_vl=drone_layers.temporalProperties()
    tempo_vl.setIsActive(True)
    tempo_vl.setStartField("date")
    tempo_vl.setMode(Qgis.VectorTemporalMode(1))
    return drone_layers
 

def gen_pathlayer():
    """
    """
    path_layers = QgsVectorLayer("LineString?crs=EPSG:4326", "temporary_lines", "memory")
    path_layers.loadNamedStyle('/home/companion/PlanSys/src/UPF4ROS2/upf4ros2_demo/upf4ros2_demo/ui/style/arrowline.qml')
    pr = path_layers.dataProvider()
    path_layers.startEditing()
    pr.addAttributes( [ QgsField("drone", QVariant.String),
                    QgsField("X1",  QVariant.Int),
                    QgsField("Y1",  QVariant.Int),
                    QgsField("date1",QVariant.DateTime),
                    QgsField("X2",  QVariant.Int),
                    QgsField("Y2",  QVariant.Int),
                   QgsField("date2",QVariant.DateTime) ] )
    path_layers.updateFields() 
    path_layers.commitChanges()
    tempo_vl=path_layers.temporalProperties()
    tempo_vl.setIsActive(True)
    tempo_vl.setStartField("date2")
    tempo_vl.setMode(Qgis.VectorTemporalMode(1))
    return path_layers




class CollectorNode(Node):
    """
    ROS2 node used to collect the position of uav at a regular interval during the simulatiion
    
    :param int n_drones: Number of drone in the current simulation
    
    """

    def __init__(self,n_drones):
        """
        """
        super().__init__('qgis_gui')
        self.drone_num_to_timestamp={i:0  for i in range(n_drones)}
        self.drone_num_to_sub_drone_pos = {i:self.create_subscription(VehicleGlobalPosition,f'vhcl{i}/fmu/vehicle_global_position/out', self.make_listener_callback_pos(i),10)  for i in range(n_drones)}
        self.drone_num_to_df={i:pd.DataFrame([],columns=["drone","x","y","datetime"])  for i in range(n_drones)} #avoid possible append at the same time
        self.df=pd.DataFrame([],columns=["drone","x","y","datetime"])

        
    
    def make_listener_callback_pos(self, ind):
        """
        Create a different callback function to subscribe to the topic of UAV position
        
        :param int ind: Index of a drone
        """
        def callback(msg):
            """
            """
            if msg.timestamp-self.drone_num_to_timestamp[ind]>5000000: #10 second: 10000000:
                current_time=datetime.datetime.now()
                current_df=self.drone_num_to_df[ind]
                current_df.loc[len(self.df.index)]=[f"d{ind}",msg.lon,msg.lat,str(current_time)]
                self.df.loc[len(self.df.index)]=[f"d{ind}",msg.lon,msg.lat,str(current_time)]
                self.drone_num_to_timestamp[ind]=msg.timestamp
                self.get_logger().info(f"d{ind}: Longitude:{msg.lon},Latitude:{msg.lat}")
        return callback
 
 
               
class CustomWind(QMainWindow):
    """
    QT window using Qgis used to represent the map and drone progression during the simulation
    
    :param list of QgsMapLayer layers: list of layer used with the first two being the drones and path layers
    :param int n_drones: Number of drone in the current simulation
    
    """
    def __init__(self,layers, n_drones):
        """
        Constructor method
        """
        QMainWindow.__init__(self)
        self.canvas = QgsMapCanvas()
        self.temporal_controller_widg = QgsTemporalControllerWidget()
        self.canvas.setTemporalController(self.temporal_controller_widg.temporalController())
        
        self.n_drones=n_drones
        self.executor=SingleThreadedExecutor()
        self.node=CollectorNode(n_drones)
        self.executor.add_node(self.node)
        
        
        self.layers=layers
        self.canvas.setDestinationCrs(layers[-1].crs())
        self.canvas.setExtent(layers[-1].extent())
        self.canvas.setLayers(layers)
        self.setCentralWidget(self.canvas)
        self.setMenuWidget(self.temporal_controller_widg)
        temporalController = self.canvas.temporalController()
        self.toolbar = self.addToolBar("Canvas actions")
        self.toolPan = QgsMapToolPan(self.canvas)
        
        self.actionPushBut = QAction("Delete data", self)
        self.actionPushBut.setCheckable(False)
        self.actionPushBut.triggered.connect(self.del_push)
        self.toolbar.addAction(self.actionPushBut)
        self.toolPan.setAction(self.actionPushBut)
        
        self.actionPushButbis = QAction("Refresh Data", self)
        self.actionPushButbis.setCheckable(False)
        self.actionPushButbis.triggered.connect(self.refresh)
        self.toolbar.addAction(self.actionPushButbis)
        self.toolPan.setAction(self.actionPushButbis)
        
        
        self.actionShowlayer = QAction("Hide Drone", self)
        self.actionShowlayer.setCheckable(True)
        self.actionShowlayer.triggered.connect(self.show_hide_drones)
        self.toolbar.addAction(self.actionShowlayer)
        self.toolPan.setAction(self.actionShowlayer)
        
        self.actionShowlayerbis = QAction("Hide Path", self)
        self.actionShowlayerbis.setCheckable(True)
        self.actionShowlayerbis.triggered.connect(self.show_hide_path)
        self.toolbar.addAction(self.actionShowlayerbis)
        self.toolPan.setAction(self.actionShowlayerbis)
        
        
        self.drone_num_to_current_data_index={i:0 for i in range(self.n_drones)}
        self.drone_num_to_previousRow={i:pd.DataFrame() for i in range(self.n_drones)}
        self.actionPushButbis.setText("Refresh Data")
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(15001)
        self.thread=Thread(target=self.executor.spin)
    
    
    def del_push(self):
        """
        Button, function used to deleted all the data 
        about the current drone and path layers
        
        """
        drone_lay=self.layers[0]
        drone_lay.startEditing()
        for feat in drone_lay.getFeatures():
            drone_lay.deleteFeature(feat.id())
        drone_lay.commitChanges()
        path_lay=self.layers[1]
        path_lay.startEditing()
        for feat in path_lay.getFeatures():
            path_lay.deleteFeature(feat.id())
        path_lay.commitChanges()
        return
    
    
    def refresh(self):
        """
        Button function used to refresh the data on the drone and path layer
        Will be executed automatically 15 seconds after last used
        
        """
        drone_lay=self.layers[0]
        path_lay=self.layers[1]
        drone_lay.startEditing()
        path_lay.startEditing() 
        for drone_num in range(self.n_drones):
            current_drone_df=self.node.drone_num_to_df[drone_num]
            current_drone_df.sort_values(by='datetime', inplace = True)
            tmpsubdf=current_drone_df.iloc[self.drone_num_to_current_data_index[drone_num]:]
            for index,row in tmpsubdf.iterrows():
                formatted = datetime.datetime.strptime(row["datetime"],'%Y-%m-%d %H:%M:%S.%f')
                newdatebis=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
                fet = QgsFeature()
                fet.setAttributes([row['drone'],row['x'],row['y'],newdatebis])
                fet.setGeometry( QgsGeometry.fromPointXY(QgsPointXY(row['x'],row['y'])) )
                drone_lay.addFeature(fet)
                if not self.drone_num_to_previousRow[drone_num].empty:
                    drone_previous_row=self.drone_num_to_previousRow[drone_num]
                    formatted_prev = datetime.datetime.strptime(drone_previous_row["datetime"],'%Y-%m-%d %H:%M:%S.%f')
                    prevdate=QDateTime(formatted.year, formatted.month, formatted.day, formatted.hour, formatted.minute, formatted.second)
                    fet = QgsFeature()
                    fet.setGeometry(QgsGeometry.fromPolylineXY([QgsPointXY(drone_previous_row['x'],drone_previous_row['y']), QgsPointXY(row['x'],row['y'])]))
                    fet.setAttributes([drone_previous_row['drone'], drone_previous_row['x'], drone_previous_row['y'], prevdate, row['x'], row['y'], newdatebis])
                    path_lay.addFeature(fet)
                self.drone_num_to_previousRow[drone_num]=row
                self.drone_num_to_current_data_index[drone_num]+=1
        path_lay.commitChanges()
        drone_lay.commitChanges()
        self.timer.start(15001)
        return 0
    
    def show_hide_drones( self, checked ):
        """
        Button function to either show or hide the drones
        """
        print(QgsProject.instance().layerTreeRoot().findLayer(self.layers[0]))
        button_layer=self.layers[0]
        if not checked:
            print("Show")
            if self.layers[0] not in self.canvas.layers():
                self.canvas.setLayers([self.layers[0]]+self.canvas.layers())
        else:
            print("Hide")
            tmp_layers=self.canvas.layers()
            tmp_layers.remove(self.layers[0])
            self.canvas.setLayers(tmp_layers)
            
    def show_hide_path( self, checked ):
        """
        Button function to either show or hide the path
        """
        print(QgsProject.instance().layerTreeRoot().findLayer(self.layers[1]))
        button_layer=self.layers[1]
        if not checked:
            print("Show")
            if self.layers[1] not in self.canvas.layers():
                self.canvas.setLayers([self.layers[1]]+self.canvas.layers())
        else:
            print("Hide")
            tmp_layers=self.canvas.layers()
            tmp_layers.remove(self.layers[1])
            self.canvas.setLayers(tmp_layers)
       

def main(args=None):
    """
    """
    rclpy.init()
    QgsApplication.setPrefixPath("/usr/", True)
    qgs=QgsApplication([],False)
    qgs.initQgis()
    
    lidar_layer_out=gen_lidarlayer()
    zone_layer_out=gen_zonelayer()
    drone_layer_out=gen_dronelayer()
    path_layer_out=gen_pathlayer()


    add_layer_toproject(lidar_layer_out,"Cloud")
    add_layer_toproject(zone_layer_out,"Vectorzone")
    add_layer_toproject(path_layer_out,"VectorLine")
    add_layer_toproject(drone_layer_out,"VectorPoint")
    
    cuswin=CustomWind([drone_layer_out,path_layer_out,zone_layer_out,lidar_layer_out],3)
    cuswin.thread.start()
    cuswin.show()

    qgs.exec()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()

