from flask import Flask
from flask_mysqldb import MySQL
from flask_restful import Resource, Api, reqparse
import json

app = Flask(__name__)
api = Api(app)

hostname = 'localhost'
db_name = 'istuvd'
db_user = 'istdbUser'
db_password = 'interface2563'

app.config['MYSQL_HOST'] = hostname
app.config['MYSQL_USER'] = db_user
app.config['MYSQL_PASSWORD'] = db_password
app.config['MYSQL_DB'] = db_name

mysql = MySQL(app)
parser = reqparse.RequestParser()

with app.app_context():
                # try:
                    resp = None
                    conn = mysql.connection
                    cur = conn.cursor()
                    cur.execute(
                        "SELECT configs  FROM `ros_options` where name = 'MONITOR'")
                    response = cur.fetchone()
                    # rospy.loginfo("Read form db:\n configs : %s",
                    #             response[0])
                    conn.commit()
                    cur.close()
                    try:
                        resp = json.loads(response[0])
                        print(resp)
                    except Exception as e:
                        #print(e)
                    
                        conn2 = mysql.connection
                        cur2 = conn2.cursor()
                        sql = ('INSERT INTO `ros_options` (name,configs,description) VALUES (%s,%s,%s)');
                        cur2.execute(sql,("MONITOR","{\"protocal\":\"Xbee\",\"host\":\"localhost\",\"port\":\"5001\"}",""))
                        # response = cur2.fetchone()
                        # print(response)
                        conn2.commit()
                        cur2.close()
                        print("ADD SUCCESS")
                # except Exception as e:
                #     print(e)
