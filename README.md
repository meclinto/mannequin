# mannequin
A Gazebo simulation of a mannequin based on anthropometric data for a woman in the 50th percentile, taken from "Kodak's Ergonomic Design for People at Work," which includes a compilation of peer reviewed studies measuring the human body. Additionally, the body segment weights are based on data from "Weight, Volume, and Center of Mass of Segments of the Human Body" (1969) by Charles E. Clauser et. al. 

To see the break down of body measurements, see /models/mannequin/measurement_breakdown. For a list of the body part masses and inertias, see /body_segment_intertias. 

Presently, the model has some instability during high angle actuation, mostly around the arms. It is easily fully stabilized by removing the elbow pronation and wrist deviation links/joints however. In the future, I want to improve the inertial and collision definitions of the model, as well as fine tune the PID controllers, as I believe this is what needed to stabilize these joints with their full, anatomically accurate DOF at these joints.
