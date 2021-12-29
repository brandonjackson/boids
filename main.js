/**
 * BOIDS SIMULATION
 * The Algorithm
 *   - separation: steer to avoid crowding local flockmates
 *   - alignment: steer towards the average heading of local flockmates
 *   - cohesion: steer to move towards the average position (center of mass) of local flockmates
 */

// sizes must be divisible by 100
 const BIN_SIZE = 100
 const CANVAS_WIDTH = 900
 const CANVAS_HEIGHT = 600
 const N_BOIDS = 100
 const DISPLAY_SIZE = 5
 
 const SPEED = 2
 const BOUNDARY_SIZE = DISPLAY_SIZE * 4
 const SEPARATION_RADIUS = DISPLAY_SIZE * 4
 const LOCAL_RADIUS =  100
 const FORCE_SEPARATION = 0.03
 const FORCE_ALIGNMENT = 0.05
 const FORCE_COHESION = 0.02
 
 const BIRD_VISION = true
 const BIRD_MAX_TRACKING = 7
 const BIRD_FOV = 161
  
 const DEBUG_VIZ = false
 
 let boids = []
 let distanceMatrix = {}
 
 
 function setup() {
     angleMode(DEGREES);
     background(230);
     createCanvas(CANVAS_WIDTH, CANVAS_HEIGHT);
 
     for(let i = 0; i < N_BOIDS; i++){
         boids.push(new Boid())
     }

     let localBoids = new BinLattice(boids,)
     let nearbyBoidZero = lattice.getNearbyIds(LOCAL_RADIUS, boids[0].position.x, boids[0].position.y)
     console.log(nearbyBoidZero)
 }
 
 function draw() {
    clear()
    distanceMatrix = computeDistanceMatrix(boids)
 
     for(let i = 0; i < boids.length; i++){
        boids[i].update(boids)
        boids[i].display()
     }
     
 }

 class BinLattice {
     constructor(boids){
         this.binsX = CANVAS_WIDTH / BIN_SIZE
         this.binsY = CANVAS_HEIGHT / BIN_SIZE
         this.bins = []
         for(let i = 0; i < this.binsX; i++){
            this.bins[i] = []
             for(let j = 0; j < this.binsY; j++){
                 this.bins[i][j] = []
             }
         }
         for(let boidIndex = 0; boidIndex < boids.length; boidIndex++){
             this.updateMembership(boids[boidIndex].id, boids[boidIndex].position.x, boids[boidIndex].position.y)
         }
     }
     pos2bin(x, y){
        let myBinX = Math.floor(x / BIN_SIZE)
        let myBinY = Math.floor(y / BIN_SIZE)
        return { binX: myBinX, binY: myBinY }
    }
     updateMembership(id, x, y){
         let bins = this.pos2bin(x, y)
         this.bins[bins.binX][bins.binY].push(id)
     }
     getNearbyIds(radius, x, y){
        // console.log("boid at " + x + ", " + y)
        let ids = []
         let nwBins = this.pos2bin(x - radius, y - radius)
         let seBins = this.pos2bin(x + radius, y + radius)
         for(let i = nwBins.binX; i <= seBins.binX; i++){
             for(let j = nwBins.binY; j <= seBins.binY; j++){
                //  console.log("getting boids in bin " + i + ", " + j)
                 if(i > 0 && j > 0 && i < this.binsX && j < this.binsY && this.bins[i][j].length > 0){
                    ids = ids.concat(this.bins[i][j])
                 }
             }
         }
         return ids
     }
 }

 function computeDistanceMatrix(boids){
     // SHORTCUT
     // we'll use MAX_RADIUS to ignore anything that is beyond a bird's distance from the start
     const MAX_RADIUS = Math.max(SEPARATION_RADIUS, LOCAL_RADIUS)

     let distances = {}
     for(let i = 0; i < boids.length; i++){
        distances[boids[i].id] = {}
     }

     for(let i = 0; i < boids.length; i++){
        distances[boids[i].id] = {}
        for(let j = 0; j < boids.length; j++){
            if(distances[boids[i].id][boids[j].id] != undefined){
                continue
            }
            let d = boids[i].position.dist(boids[j].position)
            if(d > MAX_RADIUS || boids[i].id == boids[j].id){
                distances[boids[i].id][boids[j].id] = null;
                distances[boids[j].id][boids[i].id] = null;
            } else {
                distances[boids[i].id][boids[j].id] = d;
                distances[boids[j].id][boids[i].id] = d;
            }
        }
     }
     return distances

 }
 
 
 /*
 *
 * Decision 1: Store angle or velocity components? 
 *       - How would a bird do it? A bird would definitely prioritise angle of local birds rather than care about degree of harmony.
 *       - decided to add both heading and velocity components for now...
 *       - UPDATE impossible to do without velocity components lol
 *
 * Local radius vs max trackable
 *       - i think having a max trackable is better because it means that birds who only see one thing but are far away will still hone in
 *       - i think this might be a way to actually get the murmuration to "turn" (which it frequently does in real life) because it means that 
 *         it's easier for a random peturbation to propagate
 */
 
 
 class Boid {
     constructor(){
         this.id = Math.round(Math.random() * 1000000000);
         this.position = createVector(CANVAS_WIDTH * Math.random(), CANVAS_HEIGHT * Math.random())
         this.velocity = p5.Vector.random2D().mult(SPEED)
         this.acceleration = createVector(0,0)
     }
  
     getNearbyBoids(boids, horizon){
        //  console.time('getNearbyBoids')
         let nearbyBoids = []
         for(let i = 0; i < boids.length; i++){
             if(boids[i].id == this.id){
                 continue;
             }
             if(this.position.dist(boids[i].position) <= horizon){

                 if(BIRD_VISION){
                     let angle = this.position.angleBetween(boids[i].position)
                     if(Math.abs(this.velocity.heading() - angle) >= BIRD_FOV / 2){
                         continue;
                     }
                 }    
                 nearbyBoids.push(boids[i])
             }
         }
         if(BIRD_VISION && nearbyBoids.length > BIRD_MAX_TRACKING){
             nearbyBoids.sort((a, b) => {
                 let distanceToA = distanceMatrix[a.id][this.id]
                 let distanceToB = distanceMatrix[b.id][this.id]
                 if(distanceToA < distanceToB){
                     return -1
                 }
                 if(distanceToA > distanceToB){
                     return 1
                 }
                 return 0
             });
     
             while(nearbyBoids.length - BIRD_MAX_TRACKING > 0){
                 nearbyBoids.pop()
             }
         }
        //  console.timeEnd('getNearbyBoids')
         return nearbyBoids;
     }
     
     update(boids){
 
         /*
         *   - separation: steer to avoid crowding local flockmates
         *   - alignment: steer towards the average heading of local flockmates
         *   - cohesion: steer to move towards the average position (center of mass) of local flockmates
        */
 
 
        let bounceDesiredVelocity = this.velocity.copy()
        if(this.position.x <= BOUNDARY_SIZE){
            bounceDesiredVelocity.x = SPEED
        }
        if((CANVAS_WIDTH - this.position.x) <= BOUNDARY_SIZE){
            bounceDesiredVelocity.x = -1 * SPEED
        }
        if(this.position.y <= BOUNDARY_SIZE){
            bounceDesiredVelocity.y = SPEED
        }
        if((CANVAS_HEIGHT - this.position.y) <= BOUNDARY_SIZE){
            bounceDesiredVelocity.y = -1 * SPEED
        }
        let bounceSteeringForce = p5.Vector.sub(bounceDesiredVelocity, this.velocity)

        let localBoids = this.getNearbyBoids(boids, LOCAL_RADIUS)
        let separationBoids = this.getNearbyBoids(boids, SEPARATION_RADIUS)

        // Cohesion + Alignment with the Flock
        let localCentre = createVector(0,0)
        let cohesionCount = 0;
        let localVelocity = createVector(0,0)
        for(let i = 0; i < localBoids.length; i++){

            // only "cohere" towards things not in separation zone
            if(p5.Vector.dist(this.position, localBoids[i].position) >= SEPARATION_RADIUS){
                localCentre.add(localBoids[i].position)
                cohesionCount += 1
            }
            
            localVelocity.add(localBoids[i].velocity)
        }

        let separationDesiredVelocity = this.velocity.copy()
        if(cohesionCount > 0){
            localCentre.div(cohesionCount)

            let fleeAcc = createVector()
            for(let i = 0; i < separationBoids.length; i++){
                let separation = p5.Vector.sub(this.position, separationBoids[i].position)
                let distance = separation.mag()
                fleeAcc.add(separation.normalize().div(distance))
            }
            separationDesiredVelocity = fleeAcc.normalize().mult(SPEED)
        }
        let separationSteeringForce = p5.Vector.sub(separationDesiredVelocity, this.velocity).mult(FORCE_SEPARATION)

        let cohesionDesiredVelocity = p5.Vector.sub(localCentre, this.position).setMag(SPEED)
        let cohesionSteeringForce = p5.Vector.sub(cohesionDesiredVelocity, this.velocity).mult(FORCE_COHESION)

        let alignmentDesiredVelocity = this.velocity.copy()
        if(localBoids.length > 0){
            alignmentDesiredVelocity = localVelocity.div(localBoids.length).setMag(SPEED)
        }
        
        let alignmentSteeringForce = p5.Vector.sub(alignmentDesiredVelocity, this.velocity).mult(FORCE_ALIGNMENT)

        this.acceleration.add(bounceSteeringForce)
        this.acceleration.add(alignmentSteeringForce)
        this.acceleration.add(cohesionSteeringForce)
        this.acceleration.add(separationSteeringForce)

        if(DEBUG_VIZ){
            
        // Visual Field
        stroke('#EEEEEE')//'#00FF0033')
        strokeWeight(1)
        let leftBlinker = this.velocity.copy().rotate(-1* (BIRD_FOV / 2)).setMag(LOCAL_RADIUS)
        let rightBlinker = this.velocity.copy().rotate(BIRD_FOV / 2).setMag(LOCAL_RADIUS)
        line(this.position.x, this.position.y, this.position.x + leftBlinker.x, this.position.y + leftBlinker.y)
        line(this.position.x, this.position.y, this.position.x + rightBlinker.x, this.position.y + rightBlinker.y)

        // Cohesion Force
        stroke('#00FF0066')
        strokeWeight(1)
        let cohesionForceLine = cohesionSteeringForce.copy().mult(250)
        line(this.position.x, this.position.y, this.position.x+ cohesionForceLine.x, this.position.y+ cohesionForceLine.y)

        // Cohesion Centre
        stroke('#00FF0011')
        strokeWeight(10)
        point(localCentre.x, localCentre.y)

        // Alignment Force
        stroke('#0000FF66')
        strokeWeight(1)
        let alignmentForceLine = alignmentSteeringForce.copy().mult(250)
        line(this.position.x, this.position.y, this.position.x+ alignmentForceLine.x, this.position.y+ alignmentForceLine.y)

        // Separation Force
            stroke('red')
            strokeWeight(2)
            let drawingAccelerationLine = this.acceleration.copy().mult(100)
            line(this.position.x, this.position.y, this.position.x - drawingAccelerationLine.x, this.position.y - drawingAccelerationLine.y)

        }

        this.velocity.add(this.acceleration)
    
 
         this.position = this.position.add(this.velocity);
  
         this.acceleration.mult(0)
     }
     display(){
 
         // DEBUG HELPER
         // console.log("DRAWING BOID @ (" + this.position.x + "," + this.position.y + "), heading of " + this.velocity.heading())
 
         let drawingVector = this.velocity.copy().normalize().mult(DISPLAY_SIZE)
         let lead = p5.Vector.add(this.position, drawingVector.copy().mult(sqrt(3) / 2)) // divide by 3 here to get equilateral
         let tail = p5.Vector.sub(this.position, drawingVector.copy().mult(sqrt(3) / 6))
         let leftTip = p5.Vector.add(tail, drawingVector.copy().rotate(-90).mult(0.5))
         let rightTip = p5.Vector.add(tail, drawingVector.copy().rotate(90).mult(0.5))
         
         fill(0)
         
         strokeWeight(0)
         triangle(lead.x, lead.y, leftTip.x, leftTip.y, rightTip.x, rightTip.y)
     }
 }
 