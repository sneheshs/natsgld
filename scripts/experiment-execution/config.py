'''
MIT License

Copyright (c) 2025 Snehesh Shrestha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

# -*- coding: utf-8 -*-

SCENES = { "Living Room" : ["Low Level", "Abstract"],
            "Kitchen" : ["Abstract", "Functional"],
            "Department Store" : ["High Level", "Communication"],
            "Airport" : ["Low Level", "Spatial"], 
            "Marketplace" : ["Low Level", "Functional"], 
            "Museum" : ["High Level", "Abstract"], 
            "Restaurant" : ["High Level", "Communication"] }


ACTIONS = { "Abstract" : [ "Hungry", "Peace", "Angry", "Frustrated", "Happy", "Confused", "Thinking", "Shy", "Nervous", "Cold", "Hot", "Bored", "Dry", "Humid", "Itchy", "Massage", "Loud", "Quiet", "Can’t hear", "Can’t see", "Smell", "Tired/ Sleepy/ Yawn", "Pain" ],
            "Functional" : ["Picture", "TV", "TV remote", "Pen", "Light", "Switch", "Fan", "AC", "Stove", "Microwave", "Oven", "Dishwasher", "Fridge", "Garage Door", "Door", "Locks", "Door Handle", "Washer", "Dryer", "Utensils", "Pots and pans", "Cleaning supplies", "Bug/ Insect", "Garbage/ Garbage can", "Dish rack", "Recycle bin", "Sink", "Tap", "Mirror", "Clock/ Time", "Vacuum", "Mop/ Broom", "Carpet", "Table", "Chair", "Sofa", "Keys", "Mail", "Bag/ Purse/ Backpack/ Briefcase", "Bicycle", "Car", "Tool - Scissor/ Screwdriver/ Hammer/ Wrench/ Pliers/ Allen keys/ Box cutter/ Dikes", "Medicine - Band Aid/ Pain killers", "Phone", "Pillow", "Blanket/ Throw", "Bed", "Books/ Magazine", "Glasses", "Box", "Window", "Brush", "Toothbrush", "Comb"],
            "High Level" : ["Record/ Capture <something> (Photo, video, sensor data..)", "Report status (sensor, iot device, inventory)", "Want to eat/ drink", "Check Please (restaurant)", "Need to leave/ Where’s the exit/ Door", "Locate <something>/ Show me where <something> is/ Find <something> (remote, restroom)", "Clean <something>", "Increase/ Decrease level (brightness, temperature) of <something> (light, fan, AC)", "Fetch/ Get for me", "Move <something> from A to B", "Help/ Assist in doing (move) <something> (table)", "Prep food (cut, chop, dice, wash, add, stir, …)", "Measure <something> (temperature, humidity, timer, …)", "Turn on/ Turn off/ Start/ Stop/ Play <something> (TV, music, show, movie, podcast)", "Leave me alone/ go away/ privacy…", "Read", "Write", "Tear", "Wear", "Take off", "Put on", "Type", "Selfie", "Wipe", "It’s hot/ it stinks/ Fan self", "It’s cold"],
            "Low Level" : ["Left", "Right", "Up", "Down", "Front ", "Back", "Forward", "Backwards", "Roll (left or right)", "Pitch (forward or backwards)", "Yaw (left or right)", "Lift", "Drop", "Pick up", "Throw", "Sit/ Sit down", "Stand/ Stand up", "Rub", "Shake", "Lie down", "Stop/ Hold/ Wait", "Pat/ Tap", "Twist", "Spin"],
            "Communication" : ["Yes", "No", "I understand", "I’m confused", "Greetings (Hello/ Hi)/ Bye/ Wave", "Thanks", "Sorry", "Pardon (Repeat)", "That’s good", "That’s bad/ Not good", "That’s ok/ Acceptable", "Acceptable but can be better", "Handle with care/ Gentle", "Don’t care/ Whatever", "Put some elbow grease on it/ Rough/ Vigorously ", "Pay attention/ Focus", "Look", "Listen", "May I/ Can I/ Permission/ Confirmation", "Do no touch/ Avoid", "Clap", "High Five", "Fist Bump", "FU", "Wink", "Blink", "Sign", "Shrug"],
            "Spatial" : ["Come", "Go", "Here", "There", "All over", "Small", "Big", "Relative (smaller/ larger/ more / less)", "Medium", "Little", "Lot", "Under", "Over", "Next to/ Adjacent", "Behind", "Hidden", "In front", "Visible", "Hold", "Go around"] }


REACTION_TEST = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "Kick", "Punch", "Gun", "Baseball", "Frisbee", "Basketball", "Spin", "Squat", "Touch Toe", "Tease" ]
