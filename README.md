# MeercaChaseAI
Destroying Neopets' Meerca Chase II Game with Artificial Intelligence

## Issues:
- too many turns, too little time: turns too much such that we sometimes miss the goal, need to accurately turn directly to face the goal and sometimes don't bother if we aren't turning this time round. We have to more accurately determine which direction to turn as well.
- in the red: we still hit red neggs because a-star doesn't technically avoid them, just goes around them. We can firstly see how this new algo works and determine if it doesn't bother us anymore or if we need to we can increase the range around the outer layer of the red neggs potentially increasing to the size of the meerca's body.
- slowpoke: the speed of the algorithm may be slow at times which may cause precision and accuracy issues.
