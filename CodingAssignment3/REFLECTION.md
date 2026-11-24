# Step 5 Reflection

**1. What was the hardest part for you, and what part was the easiest? How did you overcome this difficulty?**

The hardest part was getting the learned controller to actually reach the target. At first the arm moved but always stopped far away, and when I dragged the sphere far and back it got stuck with a joint error of about 3.7 rad. I added a log in IkGoalProvider that prints the IK residual, the joint tracking error and the end-effector distance every second, and that showed IK was fine (under 0.5 cm) but the arm never got to the IK joint goal. That led me to three fixes: the policy needed the goal as a baseline, IK needed to stay inside the joint range of my data, and IK had to aim at attachment_site instead of the wrist_3_link origin, which was exactly 10 cm off. The easiest part was collecting data and training the world model, which worked on the first try.

**2. How does your World Model work? What data is collected, what is your model architecture, what does your model predict?**

I collected 70,000 physics steps in Unity while ArmJointController picked a random joint goal every 3 seconds around the UR5e home pose. Each row of rollout.csv has q, qd, q* (the joint targets sent to the actuators), q' and qd'. The model is an MLP with three hidden layers of 256 that takes [q, qd, q*] (18 numbers) and predicts the normalized change to [q, qd] (12 numbers). I trained it with one-step MSE plus a 5-step unroll loss so it would not drift as fast when fed its own predictions. The one-step joint RMSE was 0.045 rad, but after 100 open-loop steps it was 0.64 rad, so I could see how errors pile up.

**3. How does your controller work? How is it trained, how does it connect to your world model, what does it control?**

The controller is a linear policy: action = goal + W·[q − goal, qd], where the action is the six joint targets for the position actuators. It is trained with CMA-ES in train_controller.py, and every fitness evaluation runs 16 imagined 200-step episodes through the world model only, with no MuJoCo in the loop. The reward penalizes joint error, velocity and big changes in action. In Unity, WorldModelController reads controller.json, computes the action every FixedUpdate and smooths it before sending it to ArmAgent. My first version did not include the goal term and could not reach goals at all, and removing the bias term fixed a constant offset of about 0.24 rad.

**4. What does your IK solver do? How does it work and how does it connect to your controller? Why and how did you pick this algorithm?**

IkGoalProvider takes the TrackingTarget position and finds six joint angles that put the UR5e tool point there. It uses damped least squares with a numerical Jacobian, where forward kinematics is done by MuJoCo itself (write q into mjData, run mj_kinematics, read the point, then restore the real state). The result goes to ArmJointController.SetGoal, and the learned controller tracks that joint goal. I picked DLS because the UR5e has 6 joints but I only solve for 3D position, so the problem has extra freedom, and the damping keeps the steps small near singular poses. I also clamp the solution to the joint range of the training data, because I found that the controller fails on goals it has never seen.

**5. How has your understanding of World Models changed while doing this assignment?**

Before this I thought the main question was how accurate the world model is. Now I think it matters just as much where the model and controller are used. My model was accurate inside the data range, but as soon as IK picked joint angles outside that range, everything built on it broke. I also saw that a controller trained only in imagination can look fine on the training return and still be wrong in a simple way, like not being able to output the goal. Watching the ghost arm drift away and get re-anchored made the open-loop error numbers feel much more real.

**6. How might you implement what you have learned here into your Term Project?**

Our term project trains a climbing agent that has to stand on boxes the physics knocks around. One idea is to collect rollouts of the climber and train a small world model that predicts whether a box will tip, then use it to check a planned step before the agent takes it. From this assignment I would keep the data range in mind, and only trust the model for states close to what we collected. I would also use something like the ghost arm and forecast line to see where the model's predictions go wrong.
