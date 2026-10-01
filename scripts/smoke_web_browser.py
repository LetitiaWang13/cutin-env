"""Optional Chromium smoke; run with uv's --with playwright dependency."""

import os
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread

from playwright.sync_api import expect, sync_playwright

from cutin.interfaces.web.server import WebServer

with TemporaryDirectory() as directory:
    server = WebServer(Path(directory), 0)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as p:
            executable = os.environ.get("PLAYWRIGHT_CHROMIUM_EXECUTABLE")
            browser = p.chromium.launch(executable_path=executable)
            page = browser.new_page(viewport={"width": 1400, "height": 1050})
            errors = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.goto(f"http://127.0.0.1:{server.server_port}/")
            expect(page.locator("#status")).to_contain_text("Ready.")
            assert page.locator("#scenario-form input[type=range]").count() == 12
            positions = [
                page.get_by_label(f"{role} x (m)", exact=True).bounding_box()
                for role in ("Ego", "Front", "Cut-in")
            ]
            assert positions[0]["x"] < positions[1]["x"] < positions[2]["x"]
            assert len({box["y"] for box in positions}) == 1
            slider = page.get_by_label("Ego speed (m/s) slider", exact=True)
            number = page.get_by_label("Ego speed (m/s)", exact=True)
            slider.fill("26.4")
            expect(number).to_have_value("26.4")
            slider.press("ArrowRight")
            expect(number).to_have_value("26.5")
            number.fill("26.425")
            expect(number).to_have_value("26.425")
            expect(slider).to_have_value("26.4")
            front = page.get_by_label("Front x (m)", exact=True)
            front.fill("1234.125")
            expect(front).to_have_value("1234.125")
            expect(page.get_by_label("Front x (m) slider", exact=True)).to_have_value(
                "200"
            )
            expect(page.locator("#offsets")).to_contain_text("+1184.13 m")
            expect(page.locator(".front .precision-hint").first).to_be_visible()
            expect(page.locator(".ego .precision-hint").nth(1)).to_be_visible()
            speeds = [
                page.get_by_label(f"{role} speed (m/s)", exact=True).bounding_box()
                for role in ("Ego", "Front", "Cut-in")
            ]
            assert len({box["y"] for box in speeds}) == 1
            assert not page.locator(".advanced").evaluate("el => el.open")
            page.locator(".advanced summary").click()
            expect(page.get_by_label("Physics Hz", exact=True)).to_have_value("30")
            expect(page.get_by_label("Controller Hz", exact=True)).to_have_value("10")
            page.locator(".advanced summary").click()
            # Number fields remain authoritative even when the quick slider
            # cannot represent the typed precision/range.
            page.get_by_role("button", name="Run and save").click()
            expect(page.locator("#status")).to_contain_text("Loaded")
            identifier = page.locator("#status").inner_text().split()[1].rstrip(".")
            saved = page.request.get(
                f"http://127.0.0.1:{server.server_port}/api/runs/{identifier}"
            ).json()
            assert saved["log"]["scenario"]["initial"]["ego"]["speed_mps"] == 26.425
            assert saved["log"]["scenario"]["initial"]["front"]["x_m"] == 1234.125
            page.get_by_role("button", name="Unsafe regression").click()
            expect(front).to_have_value("110")
            expect(page.locator(".front .precision-hint").first).to_be_hidden()
            expect(page.get_by_label("Front x (m) slider", exact=True)).to_have_value(
                "110"
            )
            page.get_by_role("button", name="Run and save").click()
            expect(page.locator("#status")).to_contain_text("Loaded")
            assert "ego collision: true" in page.locator("#assessment").inner_text()
            page.locator("#seek").fill("18")
            page.locator("#seek").dispatch_event("input")
            assert "tick 18" in page.locator("#clock").inner_text()
            if os.environ.get("CUTIN_SMOKE_SCREENSHOT"):
                page.screenshot(
                    path=os.environ["CUTIN_SMOKE_SCREENSHOT"], full_page=True
                )
            page.get_by_role("button", name="Play", exact=True).click()
            expect(page.locator("#play")).to_have_text("Play")
            assert "tick 18" in page.locator("#clock").inner_text()
            page.get_by_role("button", name="Baseline", exact=True).click()
            page.get_by_role("button", name="Run and save").click()
            expect(page.locator("#assessment")).to_contain_text("ego collision: false")
            assert page.locator("#seek").get_attribute("max") == "300"
            # Only lane pixels: ignore vehicle bodies/labels so camera motion
            # cannot pass this regression merely by moving another vehicle.
            lane_pixels = """() => {
              const ctx = document.getElementById('road').getContext('2d');
              const pixels = ctx.getImageData(0, 114, 210, 1).data;
              return Array.from({length:210}, (_, x) => pixels[x * 4] > 140);
            }"""
            initial = page.evaluate(lane_pixels)
            page.locator("#seek").fill("1")
            page.locator("#seek").dispatch_event("input")
            moved = page.evaluate(lane_pixels)
            assert initial != moved, "lane markings must move in the ego camera"
            # At 25 m/s and 30 Hz, the first-tick displacement is ~6.67 px
            # left. Avoid the subpixel transition pixels at segment boundaries.
            assert moved[6] and not initial[6]
            assert initial[25] and not moved[25]
            page.locator("#seek").fill("0")
            page.locator("#seek").dispatch_event("input")
            assert page.evaluate(lane_pixels) == initial, "seek must restore phase"
            page.reload()
            expect(page.locator("#status")).to_contain_text("Ready.")
            assert page.locator("#saved option").count() == 3
            page.set_viewport_size({"width": 390, "height": 844})
            assert page.evaluate(
                "() => document.documentElement.scrollWidth <= innerWidth"
            )
            mobile = [
                page.get_by_label(f"{role} x (m)", exact=True).bounding_box()
                for role in ("Ego", "Front", "Cut-in")
            ]
            assert len({box["y"] for box in mobile}) == 1
            page.get_by_role("button", name="Load saved run").click()
            expect(page.locator("#status")).to_contain_text("Loaded")
            assert not errors, errors
            browser.close()
            print("Chromium smoke passed: forms, metrics, replay, persistence/reload")
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
