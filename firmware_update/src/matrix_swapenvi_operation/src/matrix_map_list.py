#!/usr/bin/env python3

import rospy
import sys
import json

from matrix_msgs.srv import ListMap , ListMapResponse
from matrix_msgs.msg import MapsInfo

from flask import Flask
from flask_mysqldb import MySQL
# from flask_restful import Api

app = Flask(__name__)
# api = Api(app)
# CORS(app)

hostname = 'localhost'
db_name = 'istuvd'
db_user = 'istdbUser'
db_password = 'interface2563'

app.config['MYSQL_HOST'] = hostname
app.config['MYSQL_USER'] = db_user
app.config['MYSQL_PASSWORD'] = db_password
app.config['MYSQL_DB'] = db_name
 
mysql = MySQL(app)

class Listmaps:
    def __init__(self):
        self.maplists = None
        self.cmd = None
        self.arg = None
        self.json_data = None

        self.listmaps_server = rospy.Service(
            'matrix_listmaps',
            ListMap,
            self.listmaps_server_callback)

        self.default_map = rospy.Service(
            'matrix_listmaps/default_map',
            ListMap,
            self.default_map_callback)
        
        rospy.loginfo('Maplist Init')

    def listmaps_server_callback(self,msg):
        self.cmd = msg.cmd
        self.arg = msg.arg
        # print(msg)
        res = ListMapResponse()

        if (self.cmd == "map_list"):

            getalldata = None
            with app.app_context():
                try:
                    conn = mysql.connection
                    cur = conn.cursor()
                    cur.execute("SELECT name FROM `ros_map_server`")
                    response = cur.fetchall()

                    for i in response:
                        map_s = MapsInfo()
                        map_s.name = i[0]
                        res.maps.append(map_s)

                    res.success = True
                    
                    mysql.connection.commit()
                    cur.close()
                except Exception as e:
                    res.success = False
                    res.text = e
                    rospy.loginfo(e)
        
            return res
        
        elif (self.cmd == "get_dock_id"):
            res = ListMapResponse()
            
            getalldata = None
            with app.app_context():
                try:
                    conn = mysql.connection
                    cur = conn.cursor()
                    cur.execute("SELECT `id`,`name`,`mode`,`configs` FROM `ros_options` where `name` = 'DOCK_SETTING'")
                    response = cur.fetchall()
            

                    for i in response:
                        dumpdata = json.loads(i[3])

                        for dockdata in dumpdata:

                            if dockdata['layoutdockname'] == self.arg:

                                res.success = True
                                res.text = dockdata['dockid']
                                return res
                            else:
                                res.success = False
                    
                    mysql.connection.commit()
                    cur.close()
                    return res
                    
                except Exception as e:
                    res.success = False
                    res.text = e
                    rospy.loginfo(e)
        else:
            res.success = False
            return res

    def default_map_callback(self,msg):
        self.cmd = msg.cmd
        self.arg = msg.arg
        json_data = msg.arg
        # rospy.loginfo(msg)
        state = None
        mapname = None
        mapid = None
        res = ListMapResponse()

        self.json_data = json.loads(msg.arg)
        try:
            mapname = self.json_data["name"]
            state = "MAPNAME"
        except:
            pass
        try:
            mapid = self.json_data["id"]
            state = "MAPID"
        except:
            pass

        if (self.cmd == "default_map"):
            if(state == "MAPID"):
                rospy.loginfo(self.json_data["id"])

            
                with app.app_context():
                    try:
                        conn = mysql.connection
                        cur = conn.cursor()
                        cur.execute("UPDATE `ros_map_server` SET `is_default` = '0'")
                        cur.fetchone()
                        mysql.connection.commit()
                        cur.close()

                        cur2 = conn.cursor()
                        sql = "UPDATE `ros_map_server` SET `is_default` = '1' WHERE `id` = %s;"
                        cur2.execute(sql,[mapid]);
                        cur2.fetchone()
                        mysql.connection.commit()
                        cur2.close()

                        res.success = True
                        res.text = "MapID: "+str(mapid)+" is set_defaulted."
                        return res

                    except Exception as e:
                        res.success = False
                        res.text = e
                        rospy.loginfo(e)
            
            elif(state == "MAPNAME"):
                rospy.loginfo(self.json_data["name"])


                with app.app_context():
                    try:
                        conn = mysql.connection
                        cur = conn.cursor()
                        cur.execute("UPDATE `ros_map_server` SET `is_default` = '0'")
                        cur.fetchone()
                        mysql.connection.commit()
                        cur.close()

                        cur2 = conn.cursor()
                        sql = "UPDATE `ros_map_server` SET `is_default` = '1' WHERE `name` = %s;"
                        cur2.execute(sql,[mapname]);
                        cur2.fetchone()
                        mysql.connection.commit()
                        cur2.close()

                        res.success = True
                        res.text = "MapName: "+mapname+" is set_defaulted."
                        return res

                    except Exception as e:
                        res.success = False
                        res.text = e
                        rospy.loginfo(e)
        
        elif(self.cmd == "default_layout"):
            if(state == "MAPID"):
                rospy.loginfo(self.json_data["id"])

            
                with app.app_context():
                    try:
                        conn = mysql.connection
                        cur = conn.cursor()
                        cur.execute("UPDATE `ros_maps` SET `is_default` = '0'")
                        cur.fetchone()
                        mysql.connection.commit()
                        cur.close()

                        cur2 = conn.cursor()
                        sql = "UPDATE `ros_maps` SET `is_default` = '1' WHERE `id` = %s;"
                        cur2.execute(sql,[mapid]);
                        cur2.fetchone()
                        mysql.connection.commit()
                        cur2.close()

                        res.success = True
                        res.text = "Layout ID: "+str(mapid)+" is set_defaulted."
                        return res

                    except Exception as e:
                        res.success = False
                        res.text = e
                        rospy.loginfo(e)
            
            elif(state == "MAPNAME"):
                rospy.loginfo(self.json_data["name"])


                with app.app_context():
                    try:
                        conn = mysql.connection
                        cur = conn.cursor()
                        cur.execute("UPDATE `ros_maps` SET `is_default` = '0'")
                        cur.fetchone()
                        mysql.connection.commit()
                        cur.close()

                        cur2 = conn.cursor()
                        sql = "UPDATE `ros_maps` SET `is_default` = '1' WHERE `name` = %s;"
                        cur2.execute(sql,[mapname]);
                        cur2.fetchone()
                        mysql.connection.commit()
                        cur2.close()

                        res.success = True
                        res.text = "Layout Name: "+mapname+" is set_defaulted."
                        return res

                    except Exception as e:
                        res.success = False
                        res.text = e
                        rospy.loginfo(e)
        
        else:
            res.success = False
            return res

def main(argv=sys.argv):

    rospy.init_node('matrix_listmaps', anonymous=True)
    Listmaps()
    rospy.spin()
    
    # rospy.shutdown()


if __name__ == '__main__':
    main()
