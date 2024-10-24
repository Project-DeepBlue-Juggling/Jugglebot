// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

import React from "react";
import Grid from "@mui/material/Grid";
import FSM from "./FSM";
import TopAppBar from "./TopAppBar";

class Viewer extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      fsm_list: [],
      fsm_name_list: [],
      current_fsm_data: undefined,
      current_fsm: "ALL",
      hide_nested_fsm: false,
      show_only_active_fsms: false,
    };

    this.handle_current_fsm = this.handle_current_fsm.bind(this);
    this.handle_hide_nested_fsm = this.handle_hide_nested_fsm.bind(this);
    this.handle_show_only_active_fsms =
      this.handle_show_only_active_fsms.bind(this);
  }

  get_fsms() {
    fetch("/get_fsms")
      .then((res) => res.json())
      .then((data) => {
        let fsm_list = [];
        let fsm_name_list = ["ALL"];

        for (let key in data) {
          fsm_name_list.push(key);
          fsm_list.push(data[key]);
        }

        this.setState({
          fsm_list: fsm_list,
          fsm_name_list: fsm_name_list,
        });
      });
  }

  get_fsm() {
    fetch("/get_fsm/" + this.state.current_fsm)
      .then((res) => res.json())
      .then((data) => {
        if (Object.keys(data).length !== 0) {
          this.setState({ current_fsm_data: data });
        } else {
          this.setState({ current_fsm: "ALL" });
        }
      });
  }

  componentDidMount() {
    this.interval = setInterval(() => {
      if (this.state.current_fsm === "ALL") {
        this.get_fsms();
      } else {
        this.get_fsm();
      }
    }, 250);
  }

  componentWillUnmount() {
    clearInterval(this.interval);
  }

  handle_current_fsm(current_fsm) {
    this.setState({ current_fsm: current_fsm });
  }

  handle_hide_nested_fsm(hide_nested_fsm) {
    this.setState({ hide_nested_fsm: hide_nested_fsm });
  }

  handle_show_only_active_fsms(show_only_active_fsms) {
    this.setState({ show_only_active_fsms: show_only_active_fsms });
  }

  render() {
    return (
      <div>
        <TopAppBar
          fsm_name_list={this.state.fsm_name_list}
          handle_current_fsm={this.handle_current_fsm}
          handle_hide_nested_fsm={this.handle_hide_nested_fsm}
          handle_show_only_active_fsms={this.handle_show_only_active_fsms}
        />

        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            minHeight: "700px",
          }}
        >
          <Grid container spacing={3}>
            {this.state.current_fsm === "ALL" ? (
              this.state.fsm_list.map((fsm) => {
                if (
                  (this.state.show_only_active_fsms &&
                    fsm[0].current_state !== -1) ||
                  !this.state.show_only_active_fsms
                ) {
                  return (
                    <Grid
                      item
                      xs={6}
                      key={fsm[0].name + this.state.hide_nested_fsm}
                    >
                      <FSM
                        fsm_data={fsm}
                        alone={false}
                        hide_nested_fsm={this.state.hide_nested_fsm}
                      />
                    </Grid>
                  );
                }
              })
            ) : (
              <Grid item xs={12} key={this.state.hide_nested_fsm}>
                <FSM
                  fsm_data={this.state.current_fsm_data}
                  alone={true}
                  hide_nested_fsm={this.state.hide_nested_fsm}
                />
              </Grid>
            )}
          </Grid>
        </div>
      </div>
    );
  }
}

export default Viewer;
