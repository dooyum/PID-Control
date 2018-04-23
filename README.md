# Car Controls with PID Project Write-Up
Self-Driving Car Engineer Nanodegree Program PID Project Writeup

---

## PID Parameter Selection using Twiddle
I used the Twiddle algorithm to get a ballpark of parameter figures that would minimize the Cross Treck Error of the vehicle from the middle of the lane.

The algorithm began with P, I and D parameters all set to 0.0 and the parameter diffs, set to [0.05, 0.0005, 0.05]. THese diff values were chosen because we want to minimize the effect of the cummilative error, but highlight the present CTE and diff in CTE. After many iterations, they all converged to values of differing orders of magnitude when compared to each other.
"D" was consistently the largest of the three parameters, with "P" converging to about a tenth the magnitude of "D". The "I" parameter almost vanished when compared to the "D" parameter. It was about a hundreth of the "P" parameter.

On a scale where "D" would be a 1.0, the "P,I,D" values would end up as "0.1, 0.001, 1.0". I tried out these values and got a pretty good trail run that handle most of the road well except for the bends.

[Twiddle inpired Run](../master/videos/twiddle_pid.mov)

## Effect of each PID Coefficient
To refine the coefficient parameters that were derived from Twiddle to a working solution, I tweaked each value of the PID params independently.

### P Coefficient
The P coefficients purpose is to relatively magnify the importance of the Cross Treck Error from the present state:
 * steer_value = **-P * present_CTE** - D * (present_CTE - previous_CTE) - I * (all_previous_CTEs + present_CTE)
 
With this in mind, every new calculated CTE would have a huge impact on the steering angle if P were high and or little impact if it were too low. At 0.1, P worked mostly well but when increased by a magnitude of 10 to 1.0 it overcompensated for every new CTE calculated. This led to overcorrection on a CTE which led to an even bigger CTE, which in turn led to another overcorrection as can be seen in the video below.

[Run with increased P value](../master/videos/high_p_pid.mov)

Eventually, I increased the P value by 20% just so it could reacted slightly faster to a worsenening CTE as can be found on a bend. 1.2 was enough to ensure the vehicle did not overcorrect.

## I Coefficient
The P coefficients purpose is to relatively magnify the importance of the cumilative Cross Treck Error from all previous states:
 * steer_value = -P * present_CTE - D * (present_CTE - previous_CTE) **- I * (all_previous_CTEs + present_CTE)**
 
 This means the I coefficient magnifies the importance of the overall improvement that has been made, rather than focus on the most recent. It makes sense that this is the smallest coefficient, since it prevents overcorrection based on bad past history and gradually rewards an improving CTE. This basically encourages gradual course correction by slowly encouraging a good trend or punishing a bad one.
 To demonstrate the effect of the I coefficient, I raised its value by 100x to 0.1. This quickly went off track as the cummilative CTE rose rapidly, resulting in wild steering angles as can be seen in the video below.
 
[Run with increased I value](../master/videos/high_i_pid.mov)
 
 ## D Coefficient
The purpose of the D coefficient is to relatively magnify the importance of the cumilative Cross Treck Error based on its improvement or worsening compared to the previous CTE:
 * steer_value = -P * present_CTE **- D * (present_CTE - previous_CTE)** - I * (all_previous_CTEs + present_CTE)

This means the D coefficient magnifies the marginal improvements or regression of the CTE. This is the largest of all the parameters and a comparitvely high value doesn't result in wild swings like the other parameters because the difference between the previous and present CTE is likely to be a small fraction of the overall CTE.
To demonstrate the effect of the D parameter, I decreased it to a tenth of its value to 0.1. Since the difference in CTEs was small at the beginning of the run, it did not have much of an effect, but as the run continued the cummilative CTE and each new CTE became progressively worse because of sharper swings. This was as a result of **P** and **I** having relatively larger values which caused the overcorrection that was demonstrated above with a large **P** and **I**. This can be seen in the video below.

[Run with decreased D value](../master/videos/low_d_pid.mov)

## Tweaked PID Coefficients
After some trial and error, the **PID** params that worked best for me were **[0.12, 0.001, 1.0]**.
Here's a snippet of what that looks like.

[Final Run](../master/videos/final_pid_p.mov)
