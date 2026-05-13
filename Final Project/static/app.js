document.addEventListener("DOMContentLoaded", () => {
    const waterLevelText = document.getElementById("water-level-text");
    const waterLevelBar = document.getElementById("water-level-bar");
    const foodWeightText = document.getElementById("food-weight-text");
    const dispenseWaterBtn = document.getElementById("dispense-water-btn");
    const dispenseFoodBtn = document.getElementById("dispense-food-btn");
    const activityList = document.getElementById("activity-list");

    function logActivity(message) {
        const li = document.createElement("li");
        const time = new Date().toLocaleTimeString();
        li.textContent = `[${time}] ${message}`;
        activityList.prepend(li);
    }

    async function fetchStatus() {
        try {
            const res = await fetch("/api/status");
            const data = await res.json();

            waterLevelText.textContent = `${data.water_level} mL`;
            waterLevelBar.style.width = `${data.water_level / 500 * 100}%`;
            foodWeightText.textContent = `${data.food_weight} g`;

            const waterCard = document.getElementById('water-card');
            const waterIndicator = document.getElementById('water-indicator');
            const waterAlert = document.getElementById('water-alert');

            if (data.water_level <= 100) {
                waterCard.classList.add('warning-card');
                waterIndicator.classList.add('warning');
                waterAlert.classList.remove('hidden');
            } else {
                waterCard.classList.remove('warning-card');
                waterIndicator.classList.remove('warning');
                waterAlert.classList.add('hidden');
            }

            const foodCard = document.getElementById('food-card');
            const foodIndicator = document.getElementById('food-indicator');
            const foodAlert = document.getElementById('food-alert');

            if (data.food_weight <= 50) {
                foodCard.classList.add('warning-card');
                foodIndicator.classList.add('warning');
                foodAlert.classList.remove('hidden');
            } else {
                foodCard.classList.remove('warning-card');
                foodIndicator.classList.remove('warning');
                foodAlert.classList.add('hidden');
            }

        } catch (err) {
            console.error("Error fetching status:", err);
        }
    }

    dispenseWaterBtn.addEventListener("click", async () => {
        dispenseWaterBtn.disabled = true;
        dispenseWaterBtn.textContent = "Dispensing...";

        try {
            const res = await fetch("/api/dispense/water", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ duration: 3 })
            });
            if (res.ok) {
                logActivity("Water dispensed manually.");
            }
        } catch (err) {
            logActivity("Failed to dispense water.");
        } finally {
            dispenseWaterBtn.disabled = false;
            dispenseWaterBtn.textContent = "Dispense Water";
            fetchStatus();
        }
    });

    dispenseFoodBtn.addEventListener("click", async () => {
        dispenseFoodBtn.disabled = true;
        dispenseFoodBtn.textContent = "Dispensing...";

        try {
            const res = await fetch("/api/dispense/food", { method: "POST" });
            if (res.ok) {
                logActivity("Food dispensed manually.");
            }
        } catch (err) {
            logActivity("Failed to dispense food.");
        } finally {
            dispenseFoodBtn.disabled = false;
            dispenseFoodBtn.textContent = "Dispense Food";
            fetchStatus();
        }
    });

    // Poll status every 2 seconds
    fetchStatus();
    setInterval(fetchStatus, 2000);
});
