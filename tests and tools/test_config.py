#Write the JSON config
import json

config = {  "Serial": [
                {"number": "32",
                "baudrate": "9600",
                "end_of_frame": "0x0A0xAA0xFA",
                "data_config": [
                    ["bool","tagname1bbbb","tagvalue1bbbb","field1bbbb","offset1bbbb","multiplier1bbbb"],
                    ["bool","tagname1bbb","tagvalue1bbb","field1bbb","offset1bbb","multiplier1bbb"],
                    ["bool","tagname1bb","tagvalue1bb","field1bb","offset1bb","multiplier1bb"],
                    ["bool","tagname1b","tagvalue1b","field1b","offset1b","multiplier1b"],
                    ["uint8","tagname1","tagvalue1","field1","offset1","multiplier1"],
                    ["int8","tagname2","tagvalue2","field2","offset2","multiplier2"],
                    ["uint16","tagname3","tagvalue3","field3","offset3","multiplier3"],
                    ["int16","tagname4","tagvalue4","field4","offset4","multiplier4"],
                    ["uint32","tagname5","tagvalue5","field5","offset5","multiplier5"],
                    ["int32","tagname6","tagvalue6","field6","offset6","multiplier6"],
                    ["float","tagname7","tagvalue7","field7","offset7","multiplier7"],
                    ["half_float","tagname8","tagvalue8","field8","offset8","multiplier8"],
                    ["byte","tagname10","tagvalue10","field10","offset10","multiplier10"],
                    ["char","tagname11","tagvalue11","field11","offset11","multiplier11"],
                    ["int16","RSSi_value","tagvalue12","field12","offset12","multiplier12"],
                    ]
                }
            ],
            "Influxdb": [
                {
                "token": "",
                "org": "Taranis_project",
                "url": "http://localhost:8086",
                "bucket": "Taranis_project"
                }
            ],
            "Graphana_path": r"D:\Logiciels\grafana-v11.3.1\bin\grafana-server.exe",
            "Serial_ouput": [
                {"number": "32",
                "baudrate": "9600"
                }
            ],
            "HMI": [{
                "Display_number":"1",
                "Parameters":[{
                    "Hight":"600",
                    "Width":"800",
                    "Widget":[{
                        "3D_display":"1",
                        "Parameters":[{
                            "Hight":"600",
                            "Width":"800",
                            "Position_x":"0",
                            "Position_y":"0",
                            "scale":"1",
                            "offset_x":"0",
                            "offset_y":"0",
                            "offset_z":"0",
                            "Mesh": "Rocket.obj",
                            "Texture": "cow.png",
                            "Skybox": "34173cb38f07f89ddbebc2ac9128303f-1235-oberflex_purepapercolor_skin_grey010_detail.jpg",
                            "x":"tagname1",
                            "y":"tagname2",
                            "z":"tagname3",
                            "quat":"off",
                            "rx":"tagname4",
                            "ry":"tagname5",
                            "rz":"tagname6",
                            "rw":"tagname7",
                        "Button":"1",
                        "Parameters":[{
                            "Hight":"800",
                            "Width":"600",
                            "Position_x":"0",
                            "Position_y":"0",
                            "COM":"32",
                            "on":"outputtag1",
                            "off":"outputtag2",
                        }]  
                    }]
                }]
            }]
        }]
    }

with open('config.json', 'w') as f:
    json.dump(config, f)
