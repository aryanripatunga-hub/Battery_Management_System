from flask import Flask, jsonify
import serial

app = Flask(__name__)

ser = serial.Serial('COM5',115200,timeout=1)

latest_data = {
    "v":[0,0,0],
    "t":[0,0,0],
    "i":0,
    "p":0
}

def parse_uart(line):
    global latest_data

    try:
        data={}

        for item in line.split(','):
            key,val=item.split(':')
            data[key]=float(val)

        latest_data = {
            "v":[
                data["C1"],
                data["C2"],
                data["C3"]
            ],
            "t":[
                data["T1"],
                data["T2"],
                data["T3"]
            ],
            "i":data["I"],
            "p":data["P"]
        }

    except:
        pass


@app.route('/data')
def get_data():

    if ser.in_waiting:
        line = ser.readline().decode().strip()
        parse_uart(line)

    return jsonify(latest_data)


if __name__ == '__main__':
    app.run(host='0.0.0.0',port=5000)