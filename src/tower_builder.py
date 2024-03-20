import yaml
from geometry_msgs.msg import Pose

class TowerBuilder:
    def __init__(self, template, block_height):
        self.template = template
        self.current_layer = 0
        self.current_repeat = 0  # repetition of whole layers is possible
        self.current_block = 0
        self.total_blocks = sum((layer['repeat']+1) * len(layer['blocks']) for layer in self.template['layers'])
        self.placed_blocks = 0
        self.block_height = block_height

    def get_next_pose(self):
        layer = self.template['layers'][self.current_layer]
        block = layer['blocks'][self.current_block]
        stacks = (self.current_layer+1) * (self.current_repeat+1)

        pose = Pose()
        pose.position.x = self.template['position']['x'] + block['x']
        pose.position.y = self.template['position']['y'] + block['y']
        pose.position.z = self.template['position']['z'] + (stacks-1) * self.block_height
        pose.orientation.x = block['qx']
        pose.orientation.y = block['qy']
        pose.orientation.z = block['qz']
        pose.orientation.w = block['qw']

        return pose

    def update_tower_state(self):
        self.current_block += 1
        if self.current_block >= len(self.template['layers'][self.current_layer]['blocks']):
            self.current_block = 0
            self.current_repeat += 1
            if self.current_repeat >= self.template['layers'][self.current_layer]['repeat']+1:
                self.current_repeat = 0
                self.current_layer += 1
        self.placed_blocks += 1

    def is_tower_finished(self):
        return self.current_layer >= len(self.template['layers'])

    def reset(self):
        self.current_layer = 0
        self.current_repeat = 0
        self.current_block = 0
        self.placed_blocks = 0

    # DEBUG:
    def load_yaml_file(self, filename):
        with open(filename, 'r') as file:
            try:
                data = yaml.safe_load(file)
                return data['tower1']
            except yaml.YAMLError as e:
                print("Error loading YAML file:", e)

# DEBUG:
#tower_builder = TowerBuilder("tower1", 0.045)
#
#while not tower_builder.is_tower_finished():
#    pose = tower_builder.get_next_pose()
#    # Place cube using next_pose
#    tower_builder.update_tower_state()
