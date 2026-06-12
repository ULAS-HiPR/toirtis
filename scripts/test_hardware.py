#!/usr/bin/env python3
"""
Hardware test script for macha sensors
Tests I2C connectivity and sensor availability
"""

import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

def test_i2c():
    """Test I2C bus availability."""
    print("Testing I2C bus...")
    try:
        import board
        import busio
        i2c = busio.I2C(board.SCL, board.SDA)
        print("✓ I2C bus initialized successfully")
        i2c.deinit()
        return True
    except Exception as e:
        print(f"✗ I2C bus test failed: {e}")
        return False
#to add can test & servo test

def test_camera():
    """Test camera functionality."""
    print("Testing camera...")
    try:
        from picamera2 import Picamera2
        
        picam2 = Picamera2()
        config = picam2.create_still_configuration()
        picam2.configure(config)
        picam2.start()
        picam2.stop()
        
        print("✓ Camera initialized successfully")
        return True
    except Exception as e:
        print(f"✗ Camera test failed: {e}")
        return False

def main():
    """Run all hardware tests."""
    print("=== Toritis Hardware Test ===")
    print()
    
    tests = [
        ("I2C Bus", test_i2c),
        ("Camera", test_camera),
    ]
    
    results = []
    for name, test_func in tests:
        print(f"--- {name} ---")
        try:
            success = test_func()
            results.append((name, success))
        except Exception as e:
            print(f"✗ {name} test crashed: {e}")
            results.append((name, False))
        print()
    
    print("=== Test Summary ===")
    all_passed = True
    for name, success in results:
        status = "PASS" if success else "FAIL"
        symbol = "✓" if success else "✗"
        print(f"{symbol} {name}: {status}")
        if not success:
            all_passed = False
    
    if all_passed:
        print("\n🎉 All hardware tests passed!")
        return 0
    else:
        print("\n⚠️  Some hardware tests failed. Check connections and configuration.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
