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
            
            waterLevelText.textContent = `${data.water_level}%`;
            waterLevelBar.style.width = `${data.water_level}%`;
            foodWeightText.textContent = `${data.food_weight} g`;
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
            if(res.ok) {
                logActivity("Water dispensed manually.");
            }
        } catch(err) {
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
            if(res.ok) {
                logActivity("Food dispensed manually.");
            }
        } catch(err) {
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
