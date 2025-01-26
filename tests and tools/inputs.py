# Fonction pour détecter les ports série disponibles
def detect_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

    for x in range(0,len(portsList)):
        if portsList[x].startswith("COM" + str(val)):
    #        portVar = "COM" + str(val)

""" 
def Serial_Receiver():
    ports = serial.tools.list_ports.comports()
    serialInst = serial.Serial()
      
    portsList = []
    
    for onePort in ports:
        portsList.append(str(onePort))
        print(str(onePort))

      #val = input("Select Port: COM")

    for x in range(0,len(portsList)):
        if portsList[x].startswith("COM" + str(val)):
    #        portVar = "COM" + str(val)
"""