import xml.etree.ElementTree as ET
import random
import sys

#read the xml file that describes the map, 0 being a free space and 1 being an obstacle which is modeled like this:
# <?xml version="1.0" ?>
# <root>
#     <map>
#         <width>10</width>
#         <height>10</height>
#         <grid>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 		</grid>
#     </map>
# </root>
# then creates a new xml file, receiving as parameter the number of agents and selecting random non-repeating start and goal with the following structure:
# <?xml version="1.0" ?>
# <root>
#    <agent start_i="0" start_j="0" goal_i="7" goal_j="2"/>
#    <!-- <agent start_i="0" start_j="1" goal_i="7" goal_j="1"/> -->
#    <!-- <agent start_i="0" start_j="2" goal_i="7" goal_j="0"/> -->
# </root>

def generateRandomTask(robotNumber, amount):
    tree = ET.parse('grid_map.xml')
    root = tree.getroot()
    width = int(root.find('map').find('width').text)
    height = int(root.find('map').find('height').text)
    grid = root.find('map').find('grid')
    rows = grid.findall('row')
    #print(rows[0].text)
    #print(len(rows))
    #print(len(rows[0].text.split(" ")))
    availablePos = []
    for i in range(0, len(rows)):
        for j in range(0, len(rows[0].text.split(" "))):
            if(rows[i].text.split(" ")[j] == '0'):
                availablePos.append((i,j))
    #print(availablePos)
    for j in range(1, amount+1):  
        posCopy = availablePos.copy()
        # file = open(str(robotNumber)+' agents/grid_task_log-k2-'+str(amount+5)+'-'+str(robotNumber)+'a.xml', 'w')
        file = open("grid_task.xml", 'w')
        file.write("<?xml version=\"1.0\" ?>\r")
        file.write("<root>\r")
        for i in range(0, robotNumber):
            start = posCopy.pop(random.randint(0, len(posCopy)-1))
            goal = posCopy.pop(random.randint(0, len(posCopy)-1))
            file.write("\t<agent start_i=\"" + str(start[0]) + "\" start_j=\"" + str(start[1]) + "\" goal_i=\"" + str(goal[0]) + "\" goal_j=\"" + str(goal[1]) + "\"/>\r")
        file.write("</root>")    
        file.close()
        file_xml = ET.parse('grid_task.xml')
        root_xml = ET.ElementTree(file_xml.getroot())
        root_xml.write(str(robotNumber)+' agents/grid_task_log-k2-'+str(j+5)+'-'+str(robotNumber)+'a.xml', encoding="utf-8", xml_declaration=True)
        print(str(robotNumber)+' agents/grid_task_log-k2-'+str(j+5)+'-'+str(robotNumber)+'a.xml')
        
    
    
rnumber = sys.argv[1]
amount = sys.argv[2]
generateRandomTask(int(rnumber), int(amount))