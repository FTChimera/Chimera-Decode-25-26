// File: TeamCode2/src/main/java/org/firstinspires/ftc/teamcode2/Tests/VelocityCalcData.kt
package org.firstinspires.ftc.teamcode2.VelocityCalc

data class VelocityCalcData(
    val dist: Double,
    val velocity: Double
) {
    companion object {
        // Key value pairs for distance to velocity
        private val VEL_MAP: Map<Double, Double> = mapOf(
            // TUNE
            0.1 to 12.3,
            1.2 to 0.01,
            7.7 to 1.5,
            5.5 to 0.2
        )

        // helper to convert the map to a list of VelocityCalcData
        val VelocityCalcList: List<VelocityCalcData> = VEL_MAP.map { (k, v) -> VelocityCalcData(k, v) }
    }
}
