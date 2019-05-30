import controlP5.ControlP5
import processing.core.PApplet
import processing.core.PVector
import java.util.ArrayList

fun PApplet.random(high: Int) = random(high.toFloat())

class Sketch(private val boidsCount: Int) : PApplet() {

    companion object {
        fun run(boidsCount: Int = 150) {
            val sketch = Sketch(boidsCount)
            sketch.runSketch()
        }
    }

    private val flock = Flock()

    override fun settings() {
        size(displayWidth / 2, displayHeight / 2)
    }

    override fun setup() {
        val controller = ControlP5(this)
        controller.addSlider("Alignment", 0f, 10f, 1f, 10, height - 50, 100, 10)
        controller.addSlider("Cohesion", 0f, 10f, 1f, 10, height - 35, 100, 10)
        controller.addSlider("Separation", 0f, 10f, 1.5f, 10, height - 20, 100, 10)

        controller.addSlider("Max force", 0f, 2f, 0.4f, width - 190, height - 65, 100, 10)
        controller.addSlider("Max speed", 0f, 8f, 4f, width - 190, height - 50, 100, 10)

        val maxRadius = min(width, height) / 2f
        controller.addSlider("Perception radius", 0f, maxRadius, 50f, width - 190, height - 35, 100, 10)
        controller.addSlider("Separation radius", 0f, maxRadius, 25f, width - 190, height - 20, 100, 10)

        repeat(boidsCount) {
            flock.addBoid(
                Boid(
                    random(width), random(height)
                )
            )
        }

//        private val maxForce: Float = 0.4f,
//        private val maxSpeed: Float = 4f,
//        private val perceptionRadius: Float = 50f, // alignmentRadius and cohesionRadius
//        private val separationRadius: Float = 25f,
//        private val alignmentWeight: Float = 1f,
//        private val cohesionWeight: Float = 1f,
//        private val separationWeight: Float = 1.5f
    }

    override fun draw() {
        background(50)
        flock.run()
    }

    override fun mousePressed() {
        flock.addBoid(Boid(mouseX, mouseY))
    }

    inner class Flock {

        private val boids = ArrayList<Boid>()

        fun addBoid(boid: Boid) = boids.add(boid)

        fun run() {
            val snapshot = ArrayList(boids)
            for (boid in boids) boid.run(snapshot)
        }
    }

    inner class Boid(
        x: Float, y: Float,
        private var velocity: PVector = PVector.random2D(),
        private var acceleration: PVector = PVector(0f, 0f),
        private val sizeUnit: Float = 2f,
        private val maxForce: Float = 0.4f,
        private val maxSpeed: Float = 4f,
        private val perceptionRadius: Float = 50f, // alignmentRadius and cohesionRadius
        private val separationRadius: Float = 25f,
        private val alignmentWeight: Float = 1f,
        private val cohesionWeight: Float = 1f,
        private val separationWeight: Float = 1.5f
    ) {

        constructor(x: Int, y: Int) : this(x.toFloat(), y.toFloat())

        private var position: PVector = PVector(x, y)

        init {
            /*
            require(x >= 0f && x <= width)
            require(y >= 0f && y <= height)
            require(perceptionRadius >= separationRadius)
            require(sizeUnit > 0f)
            require(maxForce > 0f)
            require(maxSpeed > 0f)
            */
        }

        fun run(boids: ArrayList<Boid>) {
            flock(boids)
            update()
            wraparound()
            render()
        }

        private fun flock(boids: ArrayList<Boid>) {
            val behavior = steer(boids)
//            applyForce(
//                align(boids).mult(alignmentWeight),
//                cohere(boids).mult(cohesionWeight),
//                separate(boids).mult(separationWeight)
//            )
            applyForce(
                behavior.first.mult(alignmentWeight),
                behavior.second.mult(cohesionWeight),
                behavior.third.mult(separationWeight)
            )
        }

        private fun applyForce(vararg force: PVector) {
            force.forEach { acceleration.add(it) } // Σ F = m * a, with m = 1
            acceleration.limit(maxForce)
        }

        private fun update() {
            velocity.add(acceleration).limit(maxSpeed)
            position.add(velocity)
            acceleration.mult(0f)
        }

        private fun wraparound() {
            if (position.x < -sizeUnit)
                position.x = width + sizeUnit
            else if (position.x > width + sizeUnit)
                position.x = -sizeUnit

            if (position.y < -sizeUnit)
                position.y = height + sizeUnit
            else if (position.y > height + sizeUnit)
                position.y = -sizeUnit
        }

        private fun render() {
            strokeWeight(2f)
            stroke(255)

            pushMatrix() // saves the current coordinate system to the stack

            translate(position.x, position.y)
            rotate(velocity.heading() + PApplet.radians(90f))

            triangle(
                0f, -2 * sizeUnit,
                -sizeUnit, sizeUnit,
                sizeUnit, sizeUnit
            )

            popMatrix() // restores the prior coordinate system
        }

        private fun steer(boids: ArrayList<Boid>): Triple<PVector, PVector, PVector> {
            val alignment = PVector(0f, 0f)
            val cohesion = PVector(0f, 0f)
            val separation = PVector(0f, 0f)

            var count = 0f
            for (other in boids) {
                val dist = PVector.dist(position, other.position)
                if (other != this && dist < perceptionRadius && dist > 0) {
                    ++count
                    alignment.add(other.velocity)
                    cohesion.add(other.position)
                    if (dist < separationRadius)
                        separation.add(
                            PVector
                                .sub(position, other.position) // the separation force is inversely
                                .div(dist * dist)           // proportional to the square of the distance
                        )
                }
            }

            if (count > 0) {
                alignment.setMag(maxSpeed)
                alignment.sub(velocity)

                cohesion.div(count)
                cohesion.sub(position)
                cohesion.setMag(maxSpeed)
                cohesion.sub(velocity)

                separation.setMag(maxSpeed)
                separation.sub(velocity)
            }

            return Triple(alignment, cohesion, separation)
        }

        /** Separation: steer to avoid crowding local flockmates */
        private fun separate(boids: ArrayList<Boid>): PVector {
            val desiredSeparation = 25.0f
            val steer = PVector(0f, 0f, 0f)
            var count = 0
            // For every boid in the system, check if it's too close
            for (other in boids) {
                val d = PVector.dist(position, other.position)
                // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
                if (d > 0 && d < desiredSeparation) {
                    // Calculate vector pointing away from neighbor
                    val diff = PVector.sub(position, other.position)
                    diff.normalize()
                    diff.div(d)        // Weight by distance
                    steer.add(diff)
                    count++            // Keep track of how many
                }
            }
            // Average -- divide by how many
            if (count > 0) {
                steer.div(count.toFloat())
            }

            // As long as the vector is greater than 0
            if (steer.mag() > 0) {
                // First two lines of code below could be condensed with new PVector setMag() method
                // Not using this method until Processing.js catches up
                // steer.setMag(maxSpeed);

                // Implement Reynolds: Steering = Desired - Velocity
                steer.normalize()
                steer.mult(maxSpeed)
                steer.sub(velocity)
                steer.limit(maxForce)
            }
            return steer
        }

        /** Alignment: steer towards the average heading of local flockmates */
        private fun align(boids: ArrayList<Boid>): PVector {
            val neighborDist = 50f
            val sum = PVector(0f, 0f)
            var count = 0
            for (other in boids) {
                val d = PVector.dist(position, other.position)
                if (d > 0 && d < neighborDist) {
                    sum.add(other.velocity)
                    count++
                }
            }
            return if (count > 0) {
                sum.div(count.toFloat())
                // First two lines of code below could be condensed with new PVector setMag() method
                // Not using this method until Processing.js catches up
                // sum.setMag(maxSpeed);

                // Implement Reynolds: Steering = Desired - Velocity
                sum.normalize()
                sum.mult(maxSpeed)
                val steer = PVector.sub(sum, velocity)
                steer.limit(maxForce)
                steer
            } else {
                PVector(0f, 0f)
            }
        }

        /** Cohesion: steer to move toward the average position of local flockmates */
        private fun cohere(boids: ArrayList<Boid>): PVector {
            val neighborDist = 50f
            val sum = PVector(0f, 0f)   // Start with empty vector to accumulate all positions
            var count = 0
            for (other in boids) {
                val d = PVector.dist(position, other.position)
                if (d > 0 && d < neighborDist) {
                    sum.add(other.position) // Add position
                    count++
                }
            }
            return if (count > 0) {
                sum.div(count.toFloat())
                seek(sum)  // Steer towards the position
            } else {
                PVector(0f, 0f)
            }
        }


        // A method that calculates and applies a steering force towards a target
        // STEER = DESIRED MINUS VELOCITY
        private fun seek(target: PVector): PVector {
            val desired = PVector.sub(target, position)  // A vector pointing from the position to the target
            // Scale to maximum speed
            desired.normalize()
            desired.mult(maxSpeed)

            // Above two lines of code below could be condensed with new PVector setMag() method
            // Not using this method until Processing.js catches up
            // desired.setMag(maxSpeed);

            // Steering = Desired minus Velocity
            val steer = PVector.sub(desired, velocity)
            steer.limit(maxForce)  // Limit to maximum steering force
            return steer
        }
    }
}

private fun <T> Triple<T, T, T>.forEach(action: (T) -> Unit) {
    action(first)
    action(second)
    action(third)
}


fun main(args: Array<String>) {
    Sketch.run()
}