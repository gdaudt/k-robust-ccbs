import os
import xml.etree.ElementTree as ET


def indent(elem, level=0):
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
            
config2 = 'config2.xml'
config3 = 'config3.xml'
grid_task = ET.parse('grid_task.xml')
# grid_task_log = ET.parse('grid_task_log.xml')
agents = 6
replan_task = ET.parse('6 agents/grid_task_log-k2-1-6a.xml')
# gtl_root = grid_task_log.getroot()
rt_root = replan_task.getroot()
tasklog_agents = []
k_param = 'k3'

for agent_number in range(6, 14):
    for i in range(1, 11):    
        replan_task = ET.parse(str(agent_number)+' agents/grid_task_log-k2-'+str(i)+'-'+str(agent_number)+'a.xml')
        print(str(agent_number)+' agents/grid_task_log-'+k_param+'-'+str(i)+'-'+str(agent_number)+'a.xml')
        for agent in replan_task.findall('agent'):
            start_i = agent.attrib['start_i']
            start_j = agent.attrib['start_j']
            goal_i = agent.attrib['goal_i']
            goal_j = agent.attrib['goal_j']
            tasklog_agents.append([start_i, start_j, goal_i, goal_j])
        gt_root = grid_task.getroot()   
        for a in tasklog_agents:
            sub_agent = ET.Element("agent", start_i=a[0], start_j=a[1], goal_i=a[2], goal_j=a[3])
            # print('<agent start_i="' + a[0] + '" start_j="' + a[1] + '" goal_i="'+a[2]+'" goal_j="'+a[3]+'"/>')
            gt_root.append(sub_agent)
        indent(gt_root)
        # writing xml
        grid_task.write("grid_task.xml", encoding="utf-8", xml_declaration=True)

        os.system('../k-robust-ccbs grid_map.xml grid_task.xml '+config3+' >> output.txt')

        for agent in gt_root.findall('agent'):
            gt_root.remove(agent)
            
        grid_task.write("grid_task.xml", encoding="utf-8", xml_declaration=True)
        grid_task_log = ET.parse('grid_task_log.xml')
        gtl_tree = ET.ElementTree(grid_task_log.getroot())
        gtl_tree.write(str(agent_number)+' agents/'+k_param+'/grid_task_log'+k_param+'-'+str(i)+'-'+str(agent_number)+'a.xml', encoding="utf-8", xml_declaration=True)
        tasklog_agents.clear()




