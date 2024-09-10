
import numpy as np 
import subprocess
import yaml
import os
def dump_state(name='/ECM/measured_js',filename='ECM_state.yaml'):
    CUR_DIT=os.path.dirname(__file__)
    command = f"rostopic echo {name} -n1"
    process=subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
    # process.wait()
    output=process.stdout.read()[:-4]
    # print(output)
    output=yaml.load(output)
    print("position: ",output["position"])
    #save yaml file
    with open(os.path.join(CUR_DIT,filename), 'w') as file:
        yaml.dump(output, file)
    # print(output)
if __name__ == "__main__":
    dump_state()
