import os
import subprocess
from git import Repo
from git.exc import GitCommandError
from subprocess import CalledProcessError


def migrate_repositories():
    # Define the list of GitLab repository URLs
    gitlab_repo_urls = [
        # Add URLs for your other GitLab repositories here
        "https://gitlab.com/roar-gokart/arduino_comm.git",
    ]

    # GitHub repository URL
    github_repo_url = "https://github.com/UCR-FSAE/FSAE-AutonomousKarting.git"

    # Clone each GitLab repository and push it to GitHub
    for gitlab_url in gitlab_repo_urls:
        # Extract the repository name from the URL
        repo_name = gitlab_url.split("/")[-1].split(".git")[0]

        try:
            # Clone the GitLab repository
            subprocess.run(["git", "clone", gitlab_url], check=True)

            # Change directory to the cloned repository
            os.chdir(repo_name)

            # Add GitHub repository as a remote
            subprocess.run(["git", "remote", "add", "github",
                           github_repo_url], check=True)

            # subprocess.run()

            # Push to GitHub
            subprocess.run(["git", "push", "github", "develop"], check=True)

            print("Migration completed successfully.")
            return

        except (CalledProcessError, GitCommandError) as e:
            print(
                f"An error occurred during repository '{repo_name}' migration: {str(e)}")
            print("Migration failed.")
        finally:
            # Change back to the original directory and clean up (delete cloned repo)
            os.chdir("..")
            subprocess.run(["rm", "-rf", repo_name], check=True)


def main():
    migrate_repositories()


if __name__ == "__main__":
    main()
