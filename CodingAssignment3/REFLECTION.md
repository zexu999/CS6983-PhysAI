# Step 5 Reflection

**1. What were the hardest part for you, and what part was the easiest? How did you overcome this difficulty?**

The hardest part was getting the arm to actually reach the ball. At first it moved toward the ball but stopped far away, and if I dragged the ball far and back it got stuck. I added a log in IkGoalProvider that printed the IK error and the joint error every second, and the IK error was almost zero but the joint error was over 3 rad. So the problem was the controller and the goals IK was giving it, not the IK math. I also found the arm was always about 10 cm off, because IK was using the wrist body and not attachment_site. The easiest part was collecting the data and training the world model, it worked the first time.

**2. How does your World Model work? What data is collected, what is your model architecture, what does your model predict?**

I collected 70,000 steps in Unity while the arm moved to a new random joint goal every 3 seconds. Each row in rollout.csv has q, qd, the action q*, and the next q and qd. The model is an MLP with 3 hidden layers of 256. It takes q, qd and q* (18 numbers) and predicts how much q and qd change in one step (12 numbers). The one-step error was 0.045 rad, but after 100 steps on its own predictions it was 0.64 rad, so the error adds up.

**3. How does your controller work? How is it trained, how does it connect to your world model, what does it control?**

The controller is a linear policy that outputs the 6 joint targets for the actuators. It is trained with CMA-ES in train_controller.py, and the training only uses the world model, not MuJoCo. Each try runs 200 imagined steps and gets a worse score for joint error and moving too fast. My first version only used q − goal and qd, so it could never output the goal itself and the arm stopped short. I changed it to goal + W·[q − goal, qd] and removed the bias, and then the arm got to about 0.01 rad of the goal in Unity.

**4. What does your IK solver do? How does it work and how does it connect to your controller? Why and how did you pick this algorithm?**

The IK solver takes the position of the TrackingTarget sphere and finds joint angles that put the end of the arm there. I used damped least squares with a numerical Jacobian, and MuJoCo does the forward kinematics. The angles it finds are sent to ArmJointController as the goal, and the controller moves the arm to that goal. I picked DLS because it is simple and the damping stops it from jumping around near bad poses. I also limited IK to the same joint range as my training data, because when IK picked angles outside that range the controller didn't know what to do.

**5. How has your understanding of World Models changed while doing this assignment?**

Before this I thought a world model just needed to be accurate. Now I see it is only good near the data it was trained on. My model was fine inside the range I collected, but when IK sent goals outside that range the arm got stuck. I also didn't expect the prediction error to grow so much over many steps. Seeing the ghost arm drift away from the real arm made this easy to understand.

**6. How might you implement what you have learned here into your Term Project?**

In our term project an agent climbs on boxes that can move and fall. We could collect data from the climber and train a small model to guess if a box will tip before the agent steps on it. I would only trust it for situations close to the data we collected. A ghost or forecast line like in this assignment would also help us see where the model is wrong.
