# Represents a single detected robot
float32 confidence      
float32 x1              # top-left x coordinate (pixels)
float32 y1              # top-left y coordinate (pixels)
float32 x2              # bottom-right x coordinate (pixels)
float32 y2              # bottom-right y coordinate (pixels)

# Optional: Include this field ONLY if your model predicts fallen status
# based on the config key 'predictFallen' being true and 'fallenClassIndex' being valid
bool is_fallen        