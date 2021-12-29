/**
 * BOIDS SIMULATION
 * The Algorithm
 *   - separation: steer to avoid crowding local flockmates
 *   - alignment: steer towards the average heading of local flockmates
 *   - cohesion: steer to move towards the average position (center of mass) of local flockmates
 */

// sizes must be divisible by 100

let options = {}

options.N_BOIDS = 2000

options.FORCE_SEPARATION = 0.05
options.FORCE_ALIGNMENT = 0.05
options.FORCE_COHESION = 0.02

const BIN_SIZE = 50
const CANVAS_WIDTH = 1400
const CANVAS_HEIGHT = 700
const N_BOIDS = 1000
const DISPLAY_SIZE = 2

options.SPEED = 3
const BOUNDARY_SIZE = DISPLAY_SIZE * 10
const SEPARATION_RADIUS = DISPLAY_SIZE * 4
const LOCAL_RADIUS =  100

options.LIMIT_FOV = false
options.FOV = 161
options.TRACK_WINGMEN = false
options.WINGMEN = 7

options.DONUT_MODE = true
options.LOOKUP_TABLE = false

let frame = 0


options.reset = function(){
    frame = 0
    boids = []
    for(let i = 0; i < options.N_BOIDS; i++){
        boids.push(new Boid(i))
    }
    // console.log("done with reset()...dumping boids")
    // console.log(boids)
}

  
 const DEBUG_VIZ = false
 
 let boids = []
 let lattice = {}
 let distances = {}
 
 function setup() {
     angleMode(DEGREES);
     background(230);
     createCanvas(CANVAS_WIDTH, CANVAS_HEIGHT);

     var gui = new dat.GUI();
     gui.width = 330;
     
     let numberSlider = gui.add(options, 'N_BOIDS', 1, 2000)
     numberSlider.onFinishChange(function(value){
        options.reset()
     });
     gui.add(options, 'SPEED', 0, 10)
     gui.add(options, 'DONUT_MODE')
     gui.add(options, 'LOOKUP_TABLE')
     gui.add(options, 'reset')


     var f1 = gui.addFolder("Boid Forces");
   
     f1.add(options, 'FORCE_SEPARATION', 0, 0.2);
     f1.add(options, 'FORCE_ALIGNMENT', 0, 0.2);
     f1.add(options, 'FORCE_COHESION', 0, 0.2);

     var f2 = gui.addFolder("Bird Vision");      
     f2.add(options, 'LIMIT_FOV')
     f2.add(options, 'FOV', 1, 360)
     f2.add(options, 'TRACK_WINGMEN');
     f2.add(options, 'WINGMEN', 1, 100)
   
    //  f1.add(sim, 'visibleLayer',[ 'heat', 'light', 'land' ]);
    //  f1.add(sim, 'landFraction', 0, 1);
    //  var resetPlanet = f1.add(sim, 'resetPlanet');
    //  resetPlanet.name("🔄Redraw Planet");
     
    //  var f2 = gui.addFolder("Climate Controls");
    //  var temp = f2.add(sim, 'temperature',0,100).listen();
    //  temp.domElement.style.pointerEvents = "none";
    //  temp.domElement.style.opacity = .5;
    //  f2.add(sim, 'atmosophereStrength',0,1).listen();
    //  var resetClimate = f2.add(sim, 'resetClimate');
    //  resetClimate.name("🔀Randomize Heat")
     f1.open();
    //   f2.open();
   
 
    options.reset()
}
 
 function draw() {
    // console.log("frame: " + frame)
    frame+=1

    clear()
    lattice = new BinLattice(boids)
    distances = new DistanceLookup(boids)
 
     for(let i = 0; i < boids.length; i++){
        boids[i].update(boids)
        boids[i].display()
     }
     
 }

 class DistanceLookup {
     constructor(boids){
         this.distances = {}
         this.boids = boids
         for(let i = 0; i < boids.length; i++){
             this.distances[i] = Array(boids.length).fill(null)
             this.distances[i][i] = 0
         }
        }
    search(index1, index2){
        if(this.distances[index1][index2] == null){
            let distance = this.boids[index1].position.dist(this.boids[index2].position)
            this.distances[index1][index2] = distance
            this.distances[index2][index1] = distance
        }
        return this.distances[index1][index2]

    }
 }

 class BinLattice {
     constructor(boids){
         this.boids = boids
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
             this.updateMembership(boidIndex, boids[boidIndex].position.x, boids[boidIndex].position.y)
         }
     }
     pos2bin(x, y){
        let myBinX = Math.floor(x / BIN_SIZE)
        let myBinY = Math.floor(y / BIN_SIZE)
        return { binX: myBinX, binY: myBinY }
    }
     updateMembership(index, x, y){
         let bins = this.pos2bin(x, y)
         let binX = constrain(bins.binX, 0, this.binsX - 1)
         let binY = constrain(bins.binY, 0, this.binsY - 1) // constrain in case boid is off screen for some reason
        //  if(this.bins[binX][binY] == undefined){
        //     console.error('bin undefined, dumpin max, then real, bin indices')
        //     console.log(this.binsX)
        //     console.log(this.binsY)
        //     console.log(binX)
        //     console.log(binY)
        //     console.log(this.bins)
        //  }
         
         this.bins[binX][binY].push(index)
     }
     getNearbyBoids(radius, x, y){
        // console.log("boid at " + x + ", " + y)
        let indices = []
         let nwBins = this.pos2bin(x - radius, y - radius)
         let seBins = this.pos2bin(x + radius, y + radius)
         for(let i = nwBins.binX; i <= seBins.binX; i++){
             for(let j = nwBins.binY; j <= seBins.binY; j++){
                 if(i >= 0 && j >= 0 && i < this.binsX && j < this.binsY && this.bins[i][j].length > 0){    
                    indices = indices.concat(this.bins[i][j])
                 }
             }
         }
         let boidsList = []
         for(let i = 0; i < indices.length; i++){
             boidsList.push(this.boids[indices[i]])
         }
         return boidsList
     }
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
     constructor(index){
         this.id = Math.round(Math.random() * 1000000000);
         this.index = index;
         this.position = createVector(CANVAS_WIDTH * Math.random(), CANVAS_HEIGHT * Math.random())
         this.velocity = p5.Vector.random2D().mult(options.SPEED)
         this.acceleration = createVector(0,0)
         if(this.position.x < 0 || this.position.x > CANVAS_WIDTH){
             console.warn("Boids created OOB!")
         }
     }
  
     getVisibleBoids(boids, horizon){
        let nearbyBoids = lattice.getNearbyBoids(horizon, this.position.x, this.position.y)
        //  console.time('getVisibleBoids')

         let visibleBoids = []
         for(let i = 0; i < nearbyBoids.length; i++){
             if(nearbyBoids[i].id == this.id){
                 continue;
             }
             if(options.LOOKUP_TABLE){
                if(distances.search(this.index, nearbyBoids[i].index) <= horizon){

                    if(options.LIMIT_FOV){
                        let angle = this.position.angleBetween(nearbyBoids[i].position)
                        if(Math.abs(this.velocity.heading() - angle) >= options.FOV / 2){
                            continue;
                        }
                    }    
                    visibleBoids.push(nearbyBoids[i])
                }   
             } else {
                if(this.position.dist(nearbyBoids[i].position) <= horizon){

                    if(options.LIMIT_FOV){
                        let angle = this.position.angleBetween(nearbyBoids[i].position)
                        if(Math.abs(this.velocity.heading() - angle) >= options.FOV / 2){
                            continue;
                        }
                    }    
                    visibleBoids.push(nearbyBoids[i])
                }
            }
         }
         if(options.TRACK_WINGMEN && visibleBoids.length > options.WINGMEN){
             visibleBoids.sort((a, b) => {
                 let distanceToA = this.position.dist(a.position)
                 let distanceToB = this.position.dist(b.position)
                 if(distanceToA < distanceToB){
                     return -1
                 }
                 if(distanceToA > distanceToB){
                     return 1
                 }
                 return 0
             });
     
             while(visibleBoids.length - options.WINGMEN > 0){
                 visibleBoids.pop()
             }
         }
        //  console.timeEnd('getvisibleBoids')
         return visibleBoids;
     }
     
     update(boids){
 
         /*
         *   - separation: steer to avoid crowding local flockmates
         *   - alignment: steer towards the average heading of local flockmates
         *   - cohesion: steer to move towards the average position (center of mass) of local flockmates
        */
 
 
        let bounceDesiredVelocity = this.velocity.copy()
        let boundaryDistanceX = CANVAS_WIDTH
        let boundaryDistanceY = CANVAS_HEIGHT
        if(this.position.x <= BOUNDARY_SIZE){
            boundaryDistanceX = this.position.x
            bounceDesiredVelocity.x = options.SPEED;
        }
        if((CANVAS_WIDTH - this.position.x) <= BOUNDARY_SIZE){
            boundaryDistanceX = (CANVAS_WIDTH - this.position.x)
            bounceDesiredVelocity.x = -1 * options.SPEED;
        }
        if(this.position.y <= BOUNDARY_SIZE){
            boundaryDistanceY = this.position.y
            bounceDesiredVelocity.y = options.SPEED;
        }
        if((CANVAS_HEIGHT - this.position.y) <= BOUNDARY_SIZE){
            boundaryDistanceY = (CANVAS_HEIGHT - this.position.y)
            bounceDesiredVelocity.y = -1 * options.SPEED;
        }
        let bounceSteeringForce = p5.Vector.sub(bounceDesiredVelocity, this.velocity).div(Math.min(boundaryDistanceX, boundaryDistanceY))

        let localBoids = this.getVisibleBoids(boids, LOCAL_RADIUS)
        let separationBoids = this.getVisibleBoids(boids, SEPARATION_RADIUS)

        // Cohesion + Alignment with the Flock
        let localCentre = createVector(0, 0)
        let cohesionCount = 0;
        let localVelocity = createVector(0,0)
        for(let i = 0; i < localBoids.length; i++){

            // only "cohere" towards things not in separation zone
            if(options.DONUT_MODE){
                if(distances.search(this.index, localBoids[i].index) >= SEPARATION_RADIUS){
                    localCentre.add(localBoids[i].position)
                    cohesionCount += 1
                }

                // if(p5.Vector.dist(this.position, localBoids[i].position) >= SEPARATION_RADIUS){
                //     localCentre.add(localBoids[i].position)
                //     cohesionCount += 1
                // }
            } else {
                localCentre.add(localBoids[i].position)
                cohesionCount += 1
            }
            
            localVelocity.add(localBoids[i].velocity)
        }

        if(cohesionCount > 0){
            localCentre.div(cohesionCount)
        } else {
            localCentre = this.position.copy()
        }

        let separationDesiredVelocity = this.velocity.copy()
        let fleeAcc = createVector()
        // console.log(separationBoids.length + " boids in separation zone")
        for(let i = 0; i < separationBoids.length; i++){
            let separation = p5.Vector.sub(this.position, separationBoids[i].position)
            let distance = separation.mag()
            fleeAcc.add(separation.normalize().div(distance))
        }
        separationDesiredVelocity = fleeAcc.normalize().mult(options.SPEED)

        let separationSteeringForce = p5.Vector.sub(separationDesiredVelocity, this.velocity).mult(options.FORCE_SEPARATION)

        let cohesionDesiredVelocity = p5.Vector.sub(localCentre, this.position).setMag(options.SPEED)
        let cohesionSteeringForce = (cohesionCount > 1) ? p5.Vector.sub(cohesionDesiredVelocity, this.velocity).mult(options.FORCE_COHESION) : createVector(0, 0)

        let alignmentDesiredVelocity = this.velocity.copy()
        if(localBoids.length > 0){
            alignmentDesiredVelocity = localVelocity.div(localBoids.length).setMag(options.SPEED)
        }
        
        let alignmentSteeringForce = p5.Vector.sub(alignmentDesiredVelocity, this.velocity).mult(options.FORCE_ALIGNMENT)

        // console.log("Bouncing:" + bounceSteeringForce.mag())
        // // console.log(bounceSteeringForce)
        // console.log("Alignment:" + alignmentSteeringForce.mag())
        // // console.log(alignmentSteeringForce)
        // console.log("Cohesion:" + cohesionSteeringForce.mag())
        // // console.log(cohesionSteeringForce)
        // console.log("Separation:" + separationSteeringForce.mag())
        // // console.log(separationSteeringForce)

        this.acceleration.add(bounceSteeringForce)
        this.acceleration.add(alignmentSteeringForce)
        this.acceleration.add(cohesionSteeringForce)
        this.acceleration.add(separationSteeringForce)

        if(DEBUG_VIZ){
            
        // Visual Field
        stroke('#EEEEEE')//'#00FF0033')
        strokeWeight(1)
        let leftBlinker = this.velocity.copy().rotate(-1* (options.FOV / 2)).setMag(LOCAL_RADIUS)
        let rightBlinker = this.velocity.copy().rotate(options.FOV / 2).setMag(LOCAL_RADIUS)
        line(this.position.x, this.position.y, this.position.x + leftBlinker.x, this.position.y + leftBlinker.y)
        line(this.position.x, this.position.y, this.position.x + rightBlinker.x, this.position.y + rightBlinker.y)

        // Steering Force
        stroke('#AAFF77')
        strokeWeight(2)
        let bounceForceLine = bounceSteeringForce.copy().mult(250)
        line(this.position.x, this.position.y, this.position.x+ bounceForceLine.x, this.position.y+ bounceForceLine.y)
        

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
 