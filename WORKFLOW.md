# Git Workflow for Contributing

We use `git` for making code changes, with a 3-branch approach:
- `main`: This branch contains only code tested on the actual robot. It's where we merge our changes after testing.
- `dev`: This is the integration branch. This is where code comes together for testing on the robot.
- `yourname-feature`: Your work. Example: `alex-shooter`, `maya-auto`, `sam-vision`

<details>
<summary>What is git or branches or GitHub?</summary>

Imagine you're writing an essay with 5 friends. Without git, you'd email files back and forth:
- `essay_final.doc`
- `essay_final_REAL.doc`
- `essay_final_REAL_v2.doc`
- `essay_final_final_edits.doc`

Git solves this by:
1. Tracking every change: You can see who changed what and when
2. Letting everyone work at once: No more "wait, are you done editing?"
3. Letting you experiment safely: Try something risky without breaking the main code
4. Time travel: Undo anything, go back to any previous version

### How it works:
```
main branch:     A --- B --- C --- D
                          \
your branch:               E --- F
```

- Each letter (A, B, C...) is a _commit_ (saved snapshot of your code)
- _Branches_ let you work on features separately
- _Merging_ combines your work back together
- _GitHub_ is a collaboration platform for git

</details>

## Daily Workflow
### Create your feature branch
When you start work on anything, first, we branch off of the `dev` branch.
```bash
git switch dev                        # change into the dev branch
git pull                              # get all latest changes
git switch -c yourname-whatyouredoing # create your feature branch
```

### Code & Commit Often
```bash
git add .
git commit -m "Describe what you changed"
git push
```
Alternatively, you can use the VS Code UI for that.

Push early and often! This backs up your work and shows your build status on GitHub.

### When Your Code Works (it builds and does what you want)
1. Push your latest changes
2. Go to GitHub and open a Pull Request to `dev`
   <img width="710" height="282" alt="CleanShot 2025-11-17 at 16 37 00@2x" src="https://github.com/user-attachments/assets/a2fa58bc-6b82-4817-9fa8-6a94350e29d3" />
3. Ask for a review in Basecamp
4. Merge it once approved

## Pull Request Guidelines

### What to include:
- **Title**: Short description (e.g. `Fix shooter speed`, `Add auto mode 2`)
- **Description**: What changed and why
- **Testing**: How you tested it (sim, robot)

## Workflow when going to competitions
1. Open a competition specific branch following the pattern `event-{eventName}`, for example `event-botb`
   - This ensures that we push on every deploy
2. When we're back from competition, merge changes into `dev` and `main`

## Common Commands Cheat Sheet
```bash
# See all branches
git branch -a

# Switch to dev and get latest
git switch dev
git pull

# Create new branch
git switch -c yourname-feature

# Save your work
git add .
git commit -m "Your message"
git push

# Get latest changes from dev into your branch
git switch yourname-feature
git pull origin dev
```

## Merge Conflicts?

If you get conflicts when merging:
1. Don't panic, this is normal
2. Pull from dev into your branch: `git pull origin dev`
3. Fix the conflicts - VSCode shows them clearly
4. Make sure that it builds and test that it still works
5. Commit and push

Ask a mentor if you're stuck!

## Quick Troubleshooting

**"My branch is behind dev"**
→ `git pull origin dev` while on your branch

**"I forgot what branch I'm on"**
→ `git branch` (the one with `*` is current)

**"I made changes on the wrong branch"**
→ `git stash`, then `git switch correct-branch`, then `git stash pop`

**"I need to undo my last commit"**
→ `git reset --soft HEAD~1` (keeps your changes)
